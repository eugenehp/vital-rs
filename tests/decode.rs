//! End-to-end decode tests over the public API, with no BLE involved.
//!
//! Every test here starts from the bytes a notification would actually carry and
//! goes through the same path [`vital::VitalClient`]'s forwarders use —
//! `decode_*` then `from_packet` — so a regression in either half shows up here
//! rather than only on hardware.

use std::str::FromStr;

use vital::impedance::ImpedanceEstimator;
use vital::parse::{decode_accelerometer, decode_battery, decode_eeg_packet};
use vital::prelude::*;
use vital::protocol::{
    parse_firmware_version, short_uuid, ACCELEROMETER_CHARACTERISTIC, BATTERY_CHARACTERISTIC,
    EEG_CHARACTERISTIC, FIRMWARE_CONTROL_CHARACTERISTIC, FIRMWARE_PAYLOAD_CHARACTERISTIC,
    FIRMWARE_VERSION_CHARACTERISTIC, IMPEDANCE_EXCITATION_HZ, IMPEDANCE_SAMPLES_PER_CYCLE,
    RENAME_CHARACTERISTIC, SENSOR_MODE_CHARACTERISTIC, UNWRITTEN_ATTRIBUTE_VALUE,
    VITAL_SERVICE_UUID,
};
use vital::types::{EegFrame, ImpedanceFrame, LeadOffStatus};

/// Build a 20-byte EEG notification.
///
/// `codes[sample][channel]`; a channel given `None` is filled with `0xFF` the way
/// the device fills channels outside the active mask.
fn eeg_packet(
    status: [u8; 2],
    codes: [[Option<i32>; EEG_CHANNELS]; EEG_SAMPLES_PER_PACKET],
) -> Vec<u8> {
    let mut bytes = status.to_vec();
    for sample in codes {
        for code in sample {
            match code {
                Some(v) => {
                    let as_u24 = (v & 0x00FF_FFFF) as u32;
                    bytes.extend_from_slice(&as_u24.to_be_bytes()[1..]);
                }
                None => bytes.extend_from_slice(&[0xFF, 0xFF, 0xFF]),
            }
        }
    }
    assert_eq!(bytes.len(), EEG_PACKET_LEN);
    bytes
}

// ── EEG ───────────────────────────────────────────────────────────────────────

#[test]
fn eeg_notification_becomes_a_scaled_frame() {
    let bytes = eeg_packet(
        [0, 0],
        [
            [Some(1), Some(-1), Some(0)],
            [Some(1000), Some(-1000), Some(8_388_607)],
        ],
    );
    let frame = EegFrame::from_packet(&decode_eeg_packet(&bytes).unwrap(), 1_234.0);

    assert_eq!(frame.timestamp, 1_234.0);
    assert_eq!(frame.channel_count(), EEG_CHANNELS);

    let ch0 = frame.channel(0).unwrap();
    assert_eq!(ch0.raw, [1, 1000]);
    assert!((ch0.uv[0] - EEG_SCALE_UV).abs() < 1e-12);
    assert!((ch0.uv[1] - 1000.0 * EEG_SCALE_UV).abs() < 1e-9);

    // Negative codes must survive the 24-bit sign extension.
    let ch1 = frame.channel(1).unwrap();
    assert_eq!(ch1.raw, [-1, -1000]);
    assert!(ch1.uv[0] < 0.0);

    // Positive full scale, the largest code the part can produce.
    assert_eq!(frame.channel(2).unwrap().raw[1], 8_388_607);
}

#[test]
fn channels_outside_the_mask_are_absent_not_minus_one() {
    // Only channel 1 streaming, as `SensorMode::EegChannel2` would give.
    let bytes = eeg_packet([0, 0], [[None, Some(5), None], [None, Some(6), None]]);
    let frame = EegFrame::from_packet(&decode_eeg_packet(&bytes).unwrap(), 0.0);

    assert_eq!(frame.channel_count(), 1);
    assert_eq!(frame.enabled().map(|(i, _)| i).collect::<Vec<_>>(), vec![1]);
    assert_eq!(frame.channel(1).unwrap().raw, [5, 6]);
    assert!(frame.channel(0).is_none());
    assert!(frame.channel(2).is_none());
}

#[test]
fn a_single_all_ones_sample_is_not_enough_to_call_a_channel_absent() {
    // One sample at -1 and one at a real value: a genuinely saturated reading
    // must not be mistaken for the device's filler.
    let bytes = eeg_packet([0, 0], [[Some(-1), None, None], [Some(42), None, None]]);
    let frame = EegFrame::from_packet(&decode_eeg_packet(&bytes).unwrap(), 0.0);
    assert_eq!(frame.channel(0).unwrap().raw, [-1, 42]);
}

#[test]
fn the_header_is_lead_off_status() {
    // Channel 0's positive electrode off, channel 2's negative electrode off.
    let bytes = eeg_packet(
        [0b001, 0b100],
        [[Some(0); EEG_CHANNELS]; EEG_SAMPLES_PER_PACKET],
    );
    let frame = EegFrame::from_packet(&decode_eeg_packet(&bytes).unwrap(), 0.0);

    assert_eq!(frame.lead_off, LeadOffStatus::from_header([0b001, 0b100]));
    assert!(frame.lead_off.positive_off(0));
    assert!(!frame.lead_off.positive_off(1));
    assert!(frame.lead_off.negative_off(2));
    assert!(frame.lead_off.any_off());
}

#[test]
fn malformed_eeg_notifications_are_rejected_not_salvaged() {
    for length in [0usize, 1, 19, 21, 40] {
        assert!(
            decode_eeg_packet(&vec![0u8; length]).is_none(),
            "{length} bytes should not decode"
        );
    }
}

// ── Impedance ─────────────────────────────────────────────────────────────────

#[test]
fn impedance_frames_demodulate_to_a_stable_amplitude() {
    // A 1000-code excitation response riding on a large electrode offset, which
    // is the realistic shape — and the thing the demodulation has to reject.
    const AMPLITUDE: f64 = 1000.0;
    const OFFSET: f64 = 250_000.0;

    let mut estimator = ImpedanceEstimator::with_averaging(8);
    let mut estimate = None;
    let mut sample = 0usize;

    for _ in 0..64 {
        let mut codes = [[None; EEG_CHANNELS]; EEG_SAMPLES_PER_PACKET];
        for slot in codes.iter_mut() {
            let phase = std::f64::consts::PI / 2.0 * sample as f64;
            let value = (AMPLITUDE * phase.cos() + OFFSET).round() as i32;
            slot[0] = Some(value);
            sample += 1;
        }
        let bytes = eeg_packet([0, 0], codes);
        let frame = ImpedanceFrame::from_packet(&decode_eeg_packet(&bytes).unwrap(), sample as f64);
        if let Some(e) = estimator.push(&frame) {
            estimate = Some(e);
            break;
        }
    }

    let estimate = estimate.expect("an estimate within 64 frames");
    let amplitude = estimate.amplitude_codes[0].expect("channel 0 measured");
    assert!(
        (amplitude - AMPLITUDE).abs() < 5.0,
        "amplitude {amplitude} should be ~{AMPLITUDE} despite the {OFFSET} offset"
    );
    // Ohms are the amplitude times a constant, and the two accessors agree.
    assert_eq!(estimate.ohms()[0], estimate.ohms_for(0));
    assert_eq!(estimate.channel_count(), 1);
    assert_eq!(estimate.worst().map(|(c, _)| c), Some(0));
    // Out-of-range channels answer None rather than panicking.
    assert!(estimate.ohms_for(99).is_none());
}

#[test]
fn excitation_is_exactly_four_samples_per_cycle() {
    // The property the demodulation relies on. If a future revision changed
    // FLEAD_OFF this would catch it before the maths silently went wrong.
    assert_eq!(IMPEDANCE_SAMPLES_PER_CYCLE, 4);
    assert!((EEG_FREQUENCY / IMPEDANCE_EXCITATION_HZ - 4.0).abs() < 1e-12);
}

// ── Accelerometer and battery ────────────────────────────────────────────────

#[test]
fn accelerometer_notification_decodes_to_g() {
    // 1 g on x, -1 count on y, zero on z, left-justified as the part sends it.
    let x: i16 = 1000 << 4;
    let y: i16 = -1 << 4;
    let mut bytes = Vec::new();
    bytes.extend_from_slice(&x.to_be_bytes());
    bytes.extend_from_slice(&y.to_be_bytes());
    bytes.extend_from_slice(&0i16.to_be_bytes());

    let raw = decode_accelerometer(&bytes).unwrap();
    let reading = AccelerometerReading {
        timestamp: 0.0,
        raw,
    };
    assert_eq!(reading.counts(), [1000, -1, 0]);
    let [gx, gy, gz] = reading.g();
    assert!((gx - 1.0).abs() < 1e-12);
    assert!((gy + 0.001).abs() < 1e-12);
    assert_eq!(gz, 0.0);

    assert!(decode_accelerometer(&bytes[..5]).is_none());
}

#[test]
fn battery_notification_decodes_fully() {
    let mv: u16 = 3985;
    let [hi, lo] = mv.to_be_bytes();
    // Charger attached (nPG low), STAT1 high, STAT2 low, so not finished.
    let bytes = [1u8, 0, 0, hi, lo, 83, 1];

    let raw = decode_battery(&bytes).unwrap();
    let reading = BatteryReading {
        timestamp: 0.0,
        raw,
    };
    assert_eq!(reading.millivolts(), mv);
    assert_eq!(reading.percent(), 83);
    assert_eq!(reading.percent_clamped(), 83);
    assert!(reading.charger_connected());
    assert!(!reading.charge_finished());
    // The device's byte and the curve recomputed here agree to a point.
    assert!((reading.percent_from_millivolts() - 83.0).abs() < 1.0);

    assert!(decode_battery(&bytes[..6]).is_none());
}

// ── Protocol surface ──────────────────────────────────────────────────────────

#[test]
fn every_characteristic_is_a_sixteen_bit_uuid_on_the_sig_base() {
    let expected = [
        (VITAL_SERVICE_UUID, 0x2D0D),
        (EEG_CHARACTERISTIC, 0x2D37),
        (SENSOR_MODE_CHARACTERISTIC, 0x2D3A),
        (RENAME_CHARACTERISTIC, 0x2D3B),
        (ACCELEROMETER_CHARACTERISTIC, 0x2D47),
        (BATTERY_CHARACTERISTIC, 0x2D57),
        (FIRMWARE_VERSION_CHARACTERISTIC, 0x2D58),
        (FIRMWARE_CONTROL_CHARACTERISTIC, 0x2D60),
        (FIRMWARE_PAYLOAD_CHARACTERISTIC, 0x2D61),
    ];
    for (uuid, assigned) in expected {
        assert_eq!(uuid, short_uuid(assigned), "0x{assigned:04X}");
        assert_eq!(
            uuid.to_string(),
            format!("0000{assigned:04x}-0000-1000-8000-00805f9b34fb")
        );
    }
    // All distinct — a copy-paste in the constants would otherwise go unnoticed.
    let mut seen: Vec<_> = expected.iter().map(|(u, _)| *u).collect();
    seen.sort();
    seen.dedup();
    assert_eq!(seen.len(), expected.len());
}

#[test]
fn sensor_mode_names_round_trip() {
    for mode in SensorMode::ALL {
        assert_eq!(SensorMode::from_str(mode.name()), Ok(mode));
        assert_eq!(mode.to_string(), mode.name());
        // And the wire encoding round-trips too.
        assert_eq!(
            SensorMode::from_command(u16::from_be_bytes(mode.encode())),
            mode
        );
    }
    // Aliases people actually type.
    assert_eq!("all".parse(), Ok(SensorMode::EegAllChannels));
    assert_eq!("imp".parse(), Ok(SensorMode::Impedance));
    assert_eq!("OFF".parse(), Ok(SensorMode::Off));
    // A bad name reports every valid one.
    let error = "wat".parse::<SensorMode>().unwrap_err().to_string();
    for mode in SensorMode::ALL {
        assert!(error.contains(mode.name()), "{error} should mention {mode}");
    }
}

#[test]
fn channel_masks_and_counts_agree() {
    for mode in SensorMode::ALL {
        let listed: Vec<usize> = mode.channels().collect();
        assert_eq!(listed.len(), mode.channel_count());
        for channel in &listed {
            assert!(mode.channel_mask() & (1 << channel) != 0);
        }
        assert!(listed.iter().all(|c| *c < EEG_CHANNELS));
    }
    assert_eq!(SensorMode::EegAllChannels.channels().count(), EEG_CHANNELS);
    assert_eq!(SensorMode::Off.channels().count(), 0);
}

#[test]
fn firmware_version_splits_version_from_build_date() {
    for (raw, version, built) in [
        ("4.3.8May  9 2022", "4.3.8", "May  9 2022"),
        ("4.3.12Jan  4 2024", "4.3.12", "Jan  4 2024"),
        ("10.20.30Dec 31 1999", "10.20.30", "Dec 31 1999"),
    ] {
        let parsed = parse_firmware_version(raw.as_bytes());
        assert_eq!(parsed.version, version);
        assert_eq!(parsed.built, built);
        assert_eq!(parsed.raw, raw);
    }

    // Trailing NULs from a fixed-width buffer are trimmed.
    let padded = parse_firmware_version(b"4.3.8May  9 2022\0\0\0\0");
    assert_eq!(padded.version, "4.3.8");

    // Too short to hold a date: kept whole rather than sliced into nonsense.
    let short = parse_firmware_version(b"4.3.8");
    assert_eq!(short.version, "4.3.8");
    assert!(short.built.is_empty());
    assert!(parse_firmware_version(b"").version.is_empty());
}

#[test]
fn the_unwritten_attribute_placeholder_is_recognisable() {
    // It is four bytes, so it can never be read as a 2-byte mode command.
    assert_eq!(UNWRITTEN_ATTRIBUTE_VALUE.len(), 4);
    assert_ne!(UNWRITTEN_ATTRIBUTE_VALUE.len(), 2);
}

#[test]
fn timing_constants_are_self_consistent() {
    assert!((EEG_SAMPLE_INTERVAL_MS - 4.0).abs() < 1e-12);
    assert!((EEG_PACKET_INTERVAL_MS - 8.0).abs() < 1e-12);
    assert!(
        (EEG_PACKET_INTERVAL_MS - EEG_SAMPLE_INTERVAL_MS * EEG_SAMPLES_PER_PACKET as f64).abs()
            < 1e-12
    );
    assert_eq!(
        EEG_PACKET_LEN,
        2 + EEG_SAMPLES_PER_PACKET * EEG_CHANNELS * 3
    );
    assert_eq!(EEG_CHANNEL_NAMES.len(), EEG_CHANNELS);
    for (channel, name) in EEG_CHANNEL_NAMES.iter().enumerate() {
        assert_eq!(eeg_channel_name(channel), *name);
    }
    assert_eq!(eeg_channel_name(EEG_CHANNELS), "?");
}

#[test]
fn firmware_update_framing_round_trips() {
    use vital::protocol::firmware;

    let framed = firmware::encode_chunk(1, &[0xDE, 0xAD, 0xBE, 0xEF]);
    assert_eq!(&framed[..4], &1u32.to_le_bytes());
    assert_eq!(&framed[4..], &[0xDE, 0xAD, 0xBE, 0xEF]);

    assert_eq!(firmware::encode_control(firmware::OTA_BEGIN), [0x00, 0x01]);
    assert_eq!(firmware::encode_control(firmware::OTA_END), [0x00, 0x02]);
    assert_eq!(
        firmware::decode_progress(&7u32.to_le_bytes()),
        Some(7),
        "progress is little-endian, unlike the control commands"
    );
    assert!(firmware::decode_progress(&[1, 2, 3]).is_none());
}
