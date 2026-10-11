//! Byte-level decoders for the device's notification formats.
//!
//! These are pure functions over slices with no BLE dependency, so they can be
//! tested against captured bytes. [`crate::VitalClient`] layers timestamps and
//! per-channel splitting on top.

use crate::protocol::{
    ACCELEROMETER_PACKET_LEN, BATTERY_PACKET_LEN, EEG_CHANNELS, EEG_HEADER_LEN, EEG_PACKET_LEN,
    EEG_SAMPLES_PER_PACKET, EEG_SCALE_UV,
};

/// One decoded EEG (or impedance) notification, losslessly.
///
/// Sample-major: `samples[s][c]` is channel `c` of sample `s`, which is the
/// order the bytes arrive in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EegPacket {
    /// The 2-byte header, uninterpreted. See [`crate::types::LeadOffStatus`].
    pub status: [u8; 2],
    /// Signed 24-bit ADC codes, sign-extended into `i32`.
    pub samples: [[i32; EEG_CHANNELS]; EEG_SAMPLES_PER_PACKET],
    /// Which channels carried real data.
    ///
    /// The device fills the slots of channels that are not in the active mask
    /// with `0xFF` bytes, so a channel whose every byte is `0xFF` in **both**
    /// samples is reported as absent.
    ///
    /// Requiring both samples to be all-ones makes a false positive need two
    /// consecutive genuine `-1` codes on the same channel, which is far less
    /// likely than one. It is still possible — a saturated or disconnected
    /// electrode can sit at full scale — so treat this as "not streaming"
    /// rather than proof.
    pub present: [bool; EEG_CHANNELS],
}

impl EegPacket {
    /// Channel `channel`'s two samples in microvolts, or `None` if that channel
    /// was not present.
    pub fn channel_uv(&self, channel: usize) -> Option<[f64; EEG_SAMPLES_PER_PACKET]> {
        if !*self.present.get(channel)? {
            return None;
        }
        Some([
            self.samples[0][channel] as f64 * EEG_SCALE_UV,
            self.samples[1][channel] as f64 * EEG_SCALE_UV,
        ])
    }

    /// Channel `channel`'s two raw codes, or `None` if that channel was not
    /// present.
    pub fn channel_raw(&self, channel: usize) -> Option<[i32; EEG_SAMPLES_PER_PACKET]> {
        if !*self.present.get(channel)? {
            return None;
        }
        Some([self.samples[0][channel], self.samples[1][channel]])
    }
}

/// Sign-extend a big-endian 24-bit two's-complement value.
///
/// The ADS1299 clocks data out MSB-first, so this is the native order.
pub const fn i24_be(bytes: [u8; 3]) -> i32 {
    let raw = ((bytes[0] as i32) << 16) | ((bytes[1] as i32) << 8) | (bytes[2] as i32);
    // Bit 23 is the sign bit of a 24-bit value; subtracting 2^24 when it is set
    // is the branch-free sign extension.
    if raw & 0x0080_0000 != 0 {
        raw - 0x0100_0000
    } else {
        raw
    }
}

/// Decode a 20-byte EEG notification.
///
/// Layout, as the device's sample task builds it:
///
/// ```text
/// byte  0..2    status header
/// byte  2..11   sample 0:  ch1[3] ch2[3] ch3[3]   24-bit big-endian, signed
/// byte 11..20   sample 1:  ch1[3] ch2[3] ch3[3]
/// ```
///
/// Returns `None` for anything that is not exactly
/// [`EEG_PACKET_LEN`] bytes — a short notification is a framing error, not a
/// partial packet to salvage.
pub fn decode_eeg_packet(data: &[u8]) -> Option<EegPacket> {
    if data.len() != EEG_PACKET_LEN {
        return None;
    }

    let status = [data[0], data[1]];
    let mut samples = [[0i32; EEG_CHANNELS]; EEG_SAMPLES_PER_PACKET];
    // All-ones is the device's filler for a channel that is switched off.
    let mut all_ones = [true; EEG_CHANNELS];

    for (s, sample) in samples.iter_mut().enumerate() {
        for (c, slot) in sample.iter_mut().enumerate() {
            let at = EEG_HEADER_LEN + (s * EEG_CHANNELS + c) * 3;
            let triple = [data[at], data[at + 1], data[at + 2]];
            if triple != [0xFF, 0xFF, 0xFF] {
                all_ones[c] = false;
            }
            *slot = i24_be(triple);
        }
    }

    let mut present = [false; EEG_CHANNELS];
    for (p, filled) in present.iter_mut().zip(all_ones) {
        *p = !filled;
    }

    Some(EegPacket {
        status,
        samples,
        present,
    })
}

/// Decode a 6-byte accelerometer notification into raw `[x, y, z]`.
///
/// Big-endian `i16` per axis. Not scaled to g — see
/// [`crate::types::AccelerometerReading::raw`].
pub fn decode_accelerometer(data: &[u8]) -> Option<[i16; 3]> {
    if data.len() != ACCELEROMETER_PACKET_LEN {
        return None;
    }
    Some([
        i16::from_be_bytes([data[0], data[1]]),
        i16::from_be_bytes([data[2], data[3]]),
        i16::from_be_bytes([data[4], data[5]]),
    ])
}

/// Decode a 7-byte battery notification.
///
/// Returns the bytes as-is; the field layout is not known. See
/// [`crate::types::BatteryReading::raw`].
pub fn decode_battery(data: &[u8]) -> Option<[u8; 7]> {
    if data.len() != BATTERY_PACKET_LEN {
        return None;
    }
    data.try_into().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn i24_sign_extension() {
        assert_eq!(i24_be([0x00, 0x00, 0x00]), 0);
        assert_eq!(i24_be([0x00, 0x00, 0x01]), 1);
        assert_eq!(i24_be([0x7F, 0xFF, 0xFF]), 8_388_607); // 2^23 - 1
        assert_eq!(i24_be([0xFF, 0xFF, 0xFF]), -1);
        assert_eq!(i24_be([0x80, 0x00, 0x00]), -8_388_608); // -2^23
    }

    /// Build a packet with distinct per-channel codes so the de-interleave
    /// order is actually pinned down, not just the length.
    fn packet(ch3_off: bool) -> Vec<u8> {
        let mut p = vec![0xAB, 0xCD];
        for sample in 0..2u8 {
            for channel in 0..3u8 {
                if channel == 2 && ch3_off {
                    p.extend_from_slice(&[0xFF, 0xFF, 0xFF]);
                } else {
                    p.extend_from_slice(&[0x00, sample, channel]);
                }
            }
        }
        p
    }

    #[test]
    fn eeg_packet_deinterleaves_sample_major() {
        let decoded = decode_eeg_packet(&packet(false)).unwrap();
        assert_eq!(decoded.status, [0xAB, 0xCD]);
        // sample 0 → low byte 0; sample 1 → low byte 1; channel in the LSB.
        assert_eq!(decoded.samples[0], [0x000000, 0x000001, 0x000002]);
        assert_eq!(decoded.samples[1], [0x000100, 0x000101, 0x000102]);
        assert_eq!(decoded.present, [true, true, true]);
    }

    #[test]
    fn disabled_channels_are_reported_absent() {
        let decoded = decode_eeg_packet(&packet(true)).unwrap();
        assert_eq!(decoded.present, [true, true, false]);
        assert!(decoded.channel_uv(2).is_none());
        assert!(decoded.channel_raw(2).is_none());
        assert!(decoded.channel_uv(0).is_some());
        // Out-of-range channel is None rather than a panic.
        assert!(decoded.channel_uv(9).is_none());
    }

    #[test]
    fn microvolt_scaling_uses_gain_one() {
        let mut p = vec![0, 0];
        // One LSB on channel 0 of sample 0, zero elsewhere. Channels 1 and 2
        // would otherwise read as all-zero (present), which is what we want.
        p.extend_from_slice(&[0x00, 0x00, 0x01]);
        p.extend_from_slice(&[0x00, 0x00, 0x00]);
        p.extend_from_slice(&[0x00, 0x00, 0x00]);
        p.extend_from_slice(&[0x00, 0x00, 0x00]);
        p.extend_from_slice(&[0x00, 0x00, 0x00]);
        p.extend_from_slice(&[0x00, 0x00, 0x00]);
        let decoded = decode_eeg_packet(&p).unwrap();
        let uv = decoded.channel_uv(0).unwrap();
        assert!((uv[0] - EEG_SCALE_UV).abs() < f64::EPSILON);
        assert_eq!(uv[1], 0.0);
    }

    #[test]
    fn wrong_lengths_are_rejected() {
        assert!(decode_eeg_packet(&[0u8; 19]).is_none());
        assert!(decode_eeg_packet(&[0u8; 21]).is_none());
        assert!(decode_eeg_packet(&[]).is_none());
        assert!(decode_accelerometer(&[0u8; 5]).is_none());
        assert!(decode_battery(&[0u8; 6]).is_none());
    }

    #[test]
    fn accelerometer_is_big_endian_signed() {
        assert_eq!(
            decode_accelerometer(&[0x01, 0x00, 0xFF, 0xFF, 0x80, 0x00]),
            Some([256, -1, -32768])
        );
    }
}
