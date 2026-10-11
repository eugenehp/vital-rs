//! Event and data types produced by [`crate::VitalClient`].

use crate::parse::EegPacket;
use crate::protocol::{
    SensorMode, ACCELEROMETER_COUNT_SHIFT, ACCELEROMETER_SCALE_G, BATTERY_MV_PER_PERCENT,
    BATTERY_ZERO_PERCENT_MV, EEG_CHANNELS, EEG_SAMPLES_PER_PACKET,
};

// ── Lead-off status ───────────────────────────────────────────────────────────

/// The ADS1299 lead-off (electrode contact) status carried in every EEG and
/// impedance notification's 2-byte header.
///
/// The front end reports contact state in its 24-bit STATUS word, where the two
/// 8-bit lead-off bytes straddle a nibble boundary
/// (`1100 | STATP[7:0] | STATN[7:0] | GPIO[3:0]`). The device un-straddles them
/// and sends the two reassembled bytes as the packet header, dropping the fixed
/// marker and the GPIO nibble.
///
/// Bit *n* corresponds to channel *n*, so only bits 0–2 are meaningful on this
/// three-channel device.
///
/// # When it carries information
///
/// Only in [`SensorMode::Impedance`], and even then only
/// [`LeadOffStatus::positive_off`]:
///
/// - In EEG mode the device writes `LOFF_SENSP = LOFF_SENSN = 0`, switching
///   detection off entirely. Expect zeros.
/// - In impedance mode it writes `LOFF_SENSP = 0x07` but leaves
///   `LOFF_SENSN = 0x00`, so the negative-input detector never runs and
///   [`LeadOffStatus::negative`] stays zero.
///
/// [`LeadOffStatus::either_off`] therefore reduces to `positive_off` in practice.
/// It exists because relying on that is relying on an implementation detail, and
/// the bit is there to be read if a future revision enables the other detector.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LeadOffStatus {
    /// `LOFF_STATP` — positive input of each channel detected as off.
    pub positive: u8,
    /// `LOFF_STATN` — negative input of each channel detected as off.
    ///
    /// Always zero on current devices; see the type docs.
    pub negative: u8,
}

impl LeadOffStatus {
    /// Interpret a packet header.
    pub const fn from_header(header: [u8; 2]) -> Self {
        Self {
            positive: header[0],
            negative: header[1],
        }
    }

    /// Whether `channel`'s positive electrode is reported off.
    pub const fn positive_off(self, channel: usize) -> bool {
        channel < EEG_CHANNELS && self.positive & (1 << channel) != 0
    }

    /// Whether `channel`'s negative electrode is reported off.
    ///
    /// Always `false` on current devices, which do not enable that detector.
    pub const fn negative_off(self, channel: usize) -> bool {
        channel < EEG_CHANNELS && self.negative & (1 << channel) != 0
    }

    /// Whether either of `channel`'s electrodes is reported off.
    pub const fn either_off(self, channel: usize) -> bool {
        self.positive_off(channel) || self.negative_off(channel)
    }

    /// Whether any channel reports an electrode off.
    pub const fn any_off(self) -> bool {
        let mask = ((1u16 << EEG_CHANNELS) - 1) as u8;
        (self.positive | self.negative) & mask != 0
    }
}

// ── EEG ───────────────────────────────────────────────────────────────────────

/// One channel's samples from one notification.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChannelSamples {
    /// Samples in microvolts.
    pub uv: [f64; EEG_SAMPLES_PER_PACKET],
    /// The same samples as signed 24-bit ADC codes.
    pub raw: [i32; EEG_SAMPLES_PER_PACKET],
}

/// One EEG notification: every enabled channel, time-aligned.
///
/// The device samples all three channels simultaneously and sends them in a
/// single 20-byte notification, so this is one frame per notification rather than
/// one event per channel. That matters: splitting a frame into per-channel events
/// would make a consumer count them back together, and it cannot know how many to
/// expect without also tracking the mode.
#[derive(Debug, Clone, PartialEq)]
pub struct EegFrame {
    /// Wall-clock milliseconds since the Unix epoch for the first sample.
    ///
    /// Extrapolated host-side from the packet count and
    /// [`crate::protocol::EEG_FREQUENCY`], anchored at the first packet.
    ///
    /// These packets carry **no device-side sequence counter**, so a dropped
    /// notification cannot be detected and shows up as a timeline that has
    /// silently slipped 8 ms. Treat this as a smoothed arrival time, not a device
    /// clock.
    pub timestamp: f64,
    /// Electrode contact status. Zero in EEG mode — see [`LeadOffStatus`].
    pub lead_off: LeadOffStatus,
    /// Per-channel samples; `None` for a channel the active mode does not stream.
    pub channels: [Option<ChannelSamples>; EEG_CHANNELS],
}

impl EegFrame {
    /// Build a frame from a decoded packet and a timestamp.
    ///
    /// Scaling and channel presence come from the packet; the timestamp is the
    /// caller's because the device does not supply one.
    ///
    /// ```
    /// use vital::parse::decode_eeg_packet;
    /// use vital::types::EegFrame;
    ///
    /// let mut bytes = vec![0x00, 0x00];
    /// bytes.extend_from_slice(&[0x00, 0x00, 0x01]); // ch0, sample 0: 1 LSB
    /// bytes.extend_from_slice(&[0x00; 15]);         // the rest zero
    /// let packet = decode_eeg_packet(&bytes).unwrap();
    ///
    /// let frame = EegFrame::from_packet(&packet, 1_000.0);
    /// assert_eq!(frame.channel_count(), 3);
    /// assert!(frame.channel(0).unwrap().uv[0] > 0.0);
    /// ```
    pub fn from_packet(packet: &EegPacket, timestamp: f64) -> Self {
        let mut channels: [Option<ChannelSamples>; EEG_CHANNELS] = Default::default();
        for (channel, slot) in channels.iter_mut().enumerate() {
            *slot = packet.channel_raw(channel).map(|raw| ChannelSamples {
                uv: packet
                    .channel_uv(channel)
                    .expect("channel_raw and channel_uv agree on presence"),
                raw,
            });
        }
        Self {
            timestamp,
            lead_off: LeadOffStatus::from_header(packet.status),
            channels,
        }
    }

    /// Samples for `channel`, if it is streaming.
    ///
    /// Returns `None` for a channel outside `0..`[`EEG_CHANNELS`] rather than
    /// panicking, so an index from elsewhere is safe to pass.
    pub fn channel(&self, channel: usize) -> Option<ChannelSamples> {
        *self.channels.get(channel)?
    }

    /// Every streaming channel, as `(index, samples)`.
    pub fn enabled(&self) -> impl Iterator<Item = (usize, ChannelSamples)> + '_ {
        self.channels
            .iter()
            .enumerate()
            .filter_map(|(i, c)| c.map(|c| (i, c)))
    }

    /// How many channels this frame carries.
    pub fn channel_count(&self) -> usize {
        self.channels.iter().flatten().count()
    }
}

/// One impedance notification: every enabled channel, time-aligned.
///
/// Same framing as [`EegFrame`], different meaning — the samples are the front
/// end's response to the lead-off excitation current. They **alternate** at
/// [`crate::protocol::IMPEDANCE_EXCITATION_HZ`], so a single sample is a point on
/// a sine wave and means little by itself. Feed frames to
/// [`crate::impedance::ImpedanceEstimator`] to get one number per electrode.
#[derive(Debug, Clone, PartialEq)]
pub struct ImpedanceFrame {
    /// Wall-clock milliseconds since the Unix epoch for the first sample.
    pub timestamp: f64,
    /// Electrode contact status — meaningful here, unlike in EEG mode.
    pub lead_off: LeadOffStatus,
    /// Per-channel raw codes; `None` for a channel that is not streaming.
    ///
    /// Not scaled: the conversion to ohms belongs to
    /// [`crate::impedance::ImpedanceEstimator`], which needs a whole cycle.
    pub channels: [Option<[i32; EEG_SAMPLES_PER_PACKET]>; EEG_CHANNELS],
}

impl ImpedanceFrame {
    /// Build a frame from a decoded packet and a timestamp.
    ///
    /// The same wire format as [`EegFrame::from_packet`]; which one to build is
    /// decided by the active [`SensorMode`], not by anything in the bytes.
    pub fn from_packet(packet: &EegPacket, timestamp: f64) -> Self {
        let mut channels = [None; EEG_CHANNELS];
        for (channel, slot) in channels.iter_mut().enumerate() {
            *slot = packet.channel_raw(channel);
        }
        Self {
            timestamp,
            lead_off: LeadOffStatus::from_header(packet.status),
            channels,
        }
    }

    /// Raw codes for `channel`, if it is streaming.
    pub fn channel(&self, channel: usize) -> Option<[i32; EEG_SAMPLES_PER_PACKET]> {
        *self.channels.get(channel)?
    }

    /// Every streaming channel, as `(index, codes)`.
    pub fn enabled(&self) -> impl Iterator<Item = (usize, [i32; EEG_SAMPLES_PER_PACKET])> + '_ {
        self.channels
            .iter()
            .enumerate()
            .filter_map(|(i, c)| c.map(|c| (i, c)))
    }
}

// ── Accelerometer ─────────────────────────────────────────────────────────────

/// One accelerometer notification — a single 3-axis sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AccelerometerReading {
    /// Wall-clock milliseconds since the Unix epoch.
    pub timestamp: f64,
    /// Raw big-endian `i16` values, in `[x, y, z]` order, straight from the
    /// LIS3DH's `OUT_[XYZ]_H`/`OUT_[XYZ]_L` pairs, data left-justified.
    pub raw: [i16; 3],
}

impl AccelerometerReading {
    /// The samples as the device treats them: 12-bit counts, sign-preserving.
    pub const fn counts(&self) -> [i16; 3] {
        [
            self.raw[0] >> ACCELEROMETER_COUNT_SHIFT,
            self.raw[1] >> ACCELEROMETER_COUNT_SHIFT,
            self.raw[2] >> ACCELEROMETER_COUNT_SHIFT,
        ]
    }

    /// The samples in g, reproducing the device's own conversion.
    ///
    /// Read [`ACCELEROMETER_SCALE_G`] before trusting the absolute magnitude: the
    /// device's scale and its `CTRL_REG4` setting disagree by a factor of two,
    /// and this follows the device.
    pub fn g(&self) -> [f64; 3] {
        let c = self.counts();
        [
            c[0] as f64 * ACCELEROMETER_SCALE_G,
            c[1] as f64 * ACCELEROMETER_SCALE_G,
            c[2] as f64 * ACCELEROMETER_SCALE_G,
        ]
    }
}

// ── Battery ───────────────────────────────────────────────────────────────────

/// One battery / charger notification.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatteryReading {
    /// Wall-clock milliseconds since the Unix epoch.
    pub timestamp: f64,
    /// The 7 bytes as received.
    ///
    /// ```text
    /// byte 0   CHG_STAT1 pin level   (GPIO 39)
    /// byte 1   CHG_STAT2 pin level   (GPIO 34)
    /// byte 2   CHG_nPG   pin level   (GPIO 4, active low)
    /// byte 3   battery millivolts, high byte
    /// byte 4   battery millivolts, low byte
    /// byte 5   state of charge, percent
    /// byte 6   charge-state flag
    /// ```
    ///
    /// Fully decoded; the accessors below are the way to read it.
    pub raw: [u8; 7],
}

impl BatteryReading {
    /// Battery voltage in millivolts.
    ///
    /// Big-endian across bytes 3 and 4, like every other multi-byte field on this
    /// device. The device reads the divider tap with `esp_adc_cal_get_voltage`
    /// and scales by 3/2, so this is the cell voltage rather than the tap.
    pub const fn millivolts(&self) -> u16 {
        u16::from_be_bytes([self.raw[3], self.raw[4]])
    }

    /// State of charge in percent, as the device reports it.
    ///
    /// The device computes a straight line from the voltage,
    /// `(millivolts - 3150) / 10`, so 3150 mV reads 0 %.
    ///
    /// # This can exceed 100
    ///
    /// The upper clamp only engages at 4200 mV, but the line already reaches 100
    /// at 4150 mV — so between 4150 and 4199 mV the device reports 101 to 104.
    /// Use [`BatteryReading::percent_clamped`] unless you specifically want the
    /// device's own number.
    pub const fn percent(&self) -> u8 {
        self.raw[5]
    }

    /// [`BatteryReading::percent`], limited to 100.
    pub const fn percent_clamped(&self) -> u8 {
        if self.raw[5] > 100 {
            100
        } else {
            self.raw[5]
        }
    }

    /// The percentage recomputed here from [`BatteryReading::millivolts`].
    ///
    /// The same curve the device applies, which makes this a consistency check
    /// on a packet rather than new information: it should equal
    /// [`BatteryReading::percent`] for every voltage below the clamp. Useful if
    /// you want a smooth value — the device's byte is integer-rounded, this is
    /// not.
    pub fn percent_from_millivolts(&self) -> f64 {
        let mv = self.millivolts() as f64;
        ((mv - BATTERY_ZERO_PERCENT_MV as f64) / BATTERY_MV_PER_PERCENT as f64).clamp(0.0, 100.0)
    }

    /// Whether a charger is plugged in.
    ///
    /// From the charger IC's `nPG` ("power good", active low) pin, so a level of
    /// zero means power is present.
    pub const fn charger_connected(&self) -> bool {
        self.raw[2] == 0
    }

    /// The charger IC's two status pins, `[STAT1, STAT2]`.
    ///
    /// The charger part is not named anywhere in the device, so these are
    /// passed through rather than given meanings.
    /// [`BatteryReading::charge_state_code`] is the device's own reading of
    /// them.
    pub const fn charge_status_pins(&self) -> [u8; 2] {
        [self.raw[0], self.raw[1]]
    }

    /// The device's charge-state flag.
    ///
    /// Despite looking like a code it is a boolean, computed from the three
    /// charger pins as:
    ///
    /// ```text
    /// 0  when nPG == 0 and STAT1 == 1 and STAT2 == 1
    /// 1  otherwise
    /// ```
    ///
    /// That is: power present, with both status outputs idle. On the common
    /// open-drain charger parts an idle status pin means "not currently pushing
    /// charge", so zero most likely means charging has terminated — but the part
    /// is unidentified, so [`BatteryReading::charge_finished`] names the condition
    /// it actually tests rather than the battery state it probably implies.
    pub const fn charge_state_code(&self) -> u8 {
        self.raw[6]
    }

    /// Whether the device's charge-state flag is clear.
    ///
    /// Equivalent to `charge_state_code() == 0`: a charger is attached and both of
    /// its status pins are idle. See [`BatteryReading::charge_state_code`] for why
    /// this is not called `is_charged`.
    pub const fn charge_finished(&self) -> bool {
        self.raw[6] == 0
    }
}

// ── Firmware ──────────────────────────────────────────────────────────────────

/// A firmware-update progress notification from the control characteristic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirmwareProgress {
    /// Count of payload chunks the device has accepted and written so far.
    pub chunks_accepted: u32,
}

// ── Events ────────────────────────────────────────────────────────────────────

/// Everything [`crate::VitalClient`] emits, in one enum.
///
/// Delivered through the `mpsc::Receiver` returned by
/// [`crate::VitalClient::connect`] or [`crate::VitalClient::connect_to`].
#[derive(Debug, Clone, PartialEq)]
pub enum VitalEvent {
    /// The link is up and the service has been discovered. Carries the device
    /// name, or its platform id when it advertises no name.
    Connected(String),
    /// One EEG notification, all enabled channels.
    Eeg(EegFrame),
    /// One impedance notification, in [`SensorMode::Impedance`].
    Impedance(ImpedanceFrame),
    /// An accelerometer sample.
    Accelerometer(AccelerometerReading),
    /// A battery / charger packet.
    Battery(BatteryReading),
    /// A firmware-update chunk count.
    FirmwareProgress(FirmwareProgress),
    /// The sensor mode changed, because something called
    /// [`crate::VitalHandle::set_mode`].
    ///
    /// Worth watching: it is what tells a consumer whether subsequent frames are
    /// [`VitalEvent::Eeg`] or [`VitalEvent::Impedance`].
    ModeChanged(SensorMode),
    /// The link dropped. No further events will arrive and the channel closes.
    Disconnected,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{BATTERY_MAX_REPORTED_PERCENT, BATTERY_ZERO_PERCENT_MV};

    #[test]
    fn lead_off_reads_per_channel_bits() {
        // Channel 0 positive off, channel 2 negative off.
        let s = LeadOffStatus::from_header([0b001, 0b100]);
        assert!(s.positive_off(0) && !s.positive_off(1) && !s.positive_off(2));
        assert!(!s.negative_off(0) && !s.negative_off(1) && s.negative_off(2));
        assert!(s.either_off(0) && !s.either_off(1) && s.either_off(2));
        assert!(s.any_off());
        assert!(!LeadOffStatus::default().any_off());
        // Bits above the channel count are not reported, even if the device sets
        // them: this is a three-channel part.
        let all = LeadOffStatus::from_header([0xFF, 0xFF]);
        assert!(!all.positive_off(EEG_CHANNELS));
        assert!(!all.either_off(7));
    }

    #[test]
    fn eeg_frame_iterates_only_streaming_channels() {
        let samples = ChannelSamples {
            uv: [1.0, 2.0],
            raw: [1, 2],
        };
        let frame = EegFrame {
            timestamp: 0.0,
            lead_off: LeadOffStatus::default(),
            channels: [Some(samples), None, Some(samples)],
        };
        assert_eq!(frame.channel_count(), 2);
        assert_eq!(
            frame.enabled().map(|(i, _)| i).collect::<Vec<_>>(),
            vec![0, 2]
        );
        assert!(frame.channel(1).is_none());
        assert!(frame.channel(99).is_none());
    }

    #[test]
    fn accelerometer_shifts_before_scaling_and_keeps_sign() {
        let a = AccelerometerReading {
            timestamp: 0.0,
            // 1 g on x at 1 mg/count, -1 count on y, 0 on z.
            raw: [1000 << 4, -1 << 4, 0],
        };
        assert_eq!(a.counts(), [1000, -1, 0]);
        let g = a.g();
        assert!((g[0] - 1.0).abs() < 1e-12);
        assert!((g[1] + 0.001).abs() < 1e-12);
        assert_eq!(g[2], 0.0);
    }

    #[test]
    fn battery_voltage_is_big_endian_millivolts() {
        // 3700 mV = 0x0E74, split high byte then low, like every other
        // multi-byte field on this device.
        let b = BatteryReading {
            timestamp: 0.0,
            raw: [1, 0, 0, 0x0E, 0x74, 55, 1],
        };
        assert_eq!(b.millivolts(), 3700);
        // The device's own curve: (3700 - 3150) / 10 = 55.
        assert_eq!(b.percent(), 55);
        assert!((b.percent_from_millivolts() - 55.0).abs() < 1e-9);
    }

    #[test]
    fn percent_can_exceed_a_hundred_and_clamping_is_opt_in() {
        // 4190 mV: past the curve's 100 % point (4150) but below the device's
        // clamp (4200), so the device really does report 104.
        let b = BatteryReading {
            timestamp: 0.0,
            raw: [0, 0, 0, 0x10, 0x5E, 104, 0],
        };
        assert_eq!(b.millivolts(), 4190);
        assert_eq!(b.percent(), 104);
        assert_eq!(b.percent(), BATTERY_MAX_REPORTED_PERCENT);
        assert_eq!(b.percent_clamped(), 100);
        // The recomputed value clamps, so it disagrees with the raw byte here —
        // by design.
        assert!((b.percent_from_millivolts() - 100.0).abs() < 1e-9);
    }

    #[test]
    fn percent_recomputation_matches_the_device_below_the_clamp() {
        // Only up to 4149 mV: that is where the curve reaches 100, and from there
        // to the device's own clamp at 4200 the device keeps counting past 100
        // while `percent_from_millivolts` does not. The two are meant to diverge
        // there — `percent_can_exceed_a_hundred_and_clamping_is_opt_in` pins that.
        for mv in BATTERY_ZERO_PERCENT_MV..=4149u16 {
            let device = ((mv - BATTERY_ZERO_PERCENT_MV) / BATTERY_MV_PER_PERCENT) as u8;
            let [hi, lo] = mv.to_be_bytes();
            let b = BatteryReading {
                timestamp: 0.0,
                raw: [0, 0, 0, hi, lo, device, 1],
            };
            assert_eq!(b.millivolts(), mv);
            // Integer truncation on the device vs. real division here, so they
            // agree to within one point.
            assert!((b.percent_from_millivolts() - device as f64).abs() < 1.0 + 1e-9);
        }
    }

    #[test]
    fn battery_charger_pins_decode() {
        // nPG is active low, so 0 means a charger is attached.
        let charging = BatteryReading {
            timestamp: 0.0,
            raw: [1, 0, 0, 0x0E, 0x74, 55, 1],
        };
        assert!(charging.charger_connected());
        assert_eq!(charging.charge_status_pins(), [1, 0]);
        assert!(!charging.charge_finished());

        // The one combination the device reports as 0: power present, both
        // status pins idle.
        let done = BatteryReading {
            timestamp: 0.0,
            raw: [1, 1, 0, 0x10, 0x68, 100, 0],
        };
        assert!(done.charge_finished());
        assert_eq!(done.charge_state_code(), 0);

        let unplugged = BatteryReading {
            timestamp: 0.0,
            raw: [0, 0, 1, 0x0E, 0x10, 50, 1],
        };
        assert!(!unplugged.charger_connected());
        assert!(!unplugged.charge_finished());
    }
}
