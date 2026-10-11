//! GATT UUIDs, sensor commands, and sampling constants for Vital Neuro headphones.
//!
//! Unlike most profiles, this device uses **16-bit assigned-number
//! UUIDs** out of the Bluetooth SIG range rather than a 128-bit
//! namespace — the device's GATT table declares them as `ESP_UUID_LEN_16`.
//! They are expanded to their 128-bit form here with [`short_uuid`], because
//! that is what the host stacks hand back from discovery.
//!
//! Values that could not be established are marked as such rather than
//! guessed at.

use uuid::Uuid;

// ── UUIDs ─────────────────────────────────────────────────────────────────────

/// Expand a 16-bit Bluetooth assigned number into its 128-bit UUID.
///
/// `0x2D0D` → `00002d0d-0000-1000-8000-00805f9b34fb`.
pub const fn short_uuid(assigned: u16) -> Uuid {
    Uuid::from_u128(0x00000000_0000_1000_8000_00805f9b34fb | ((assigned as u128) << 96))
}

/// The one primary service every characteristic lives under (`0x2D0D`).
///
/// It is named in the advertisement (`03 03 0d 2d`), so a scan can filter on it
/// and does not need to know the device's name — which is user-settable and
/// stored in the device's NVS, so there is no reliable name prefix to match.
pub const VITAL_SERVICE_UUID: Uuid = short_uuid(0x2D0D);

/// EEG sample stream (`0x2D37`) — notify, 20 bytes per notification.
///
/// Carries impedance measurements too, in the same framing, when the device is
/// in [`SensorMode::Impedance`]. See [`crate::parse::decode_eeg_packet`].
///
/// **Notify only, despite appearances.** The characteristic declaration
/// advertises the Read property, but the value attribute behind it is declared
/// with no read permission, so a read fails rather than returning the latest
/// samples. The same mismatch applies to [`ACCELEROMETER_CHARACTERISTIC`] and
/// [`BATTERY_CHARACTERISTIC`]. Subscribe; do not poll.
pub const EEG_CHARACTERISTIC: Uuid = short_uuid(0x2D37);

/// Sensor mode / enable (`0x2D3A`) — read and write, exactly 2 bytes,
/// **big-endian**.
///
/// Write a [`SensorMode`] here to select what the analogue front end does.
/// Note the endianness: this characteristic is big-endian while the CCCDs
/// around it are little-endian, as usual.
///
/// It is also readable, and the stack keeps whatever was last written, so the
/// current mode can be read back — see [`crate::VitalHandle::read_mode`]. Before
/// the first write it holds [`UNWRITTEN_ATTRIBUTE_VALUE`].
pub const SENSOR_MODE_CHARACTERISTIC: Uuid = short_uuid(0x2D3A);

/// Device rename (`0x2D3B`) — write.
///
/// The name is persisted to NVS and **the device reboots immediately**, which
/// drops the BLE link. See [`crate::VitalHandle::rename`].
pub const RENAME_CHARACTERISTIC: Uuid = short_uuid(0x2D3B);

/// Accelerometer stream (`0x2D47`) — notify, 6 bytes per notification.
pub const ACCELEROMETER_CHARACTERISTIC: Uuid = short_uuid(0x2D47);

/// Battery / charger stream (`0x2D57`) — notify, 7 bytes per notification.
pub const BATTERY_CHARACTERISTIC: Uuid = short_uuid(0x2D57);

/// Firmware version (`0x2D58`) — read only.
///
/// The device answers this read from application code rather than from the
/// attribute table, returning a fixed-width buffer of the version string
/// concatenated with the build date and no separator. See
/// [`parse_firmware_version`].
pub const FIRMWARE_VERSION_CHARACTERISTIC: Uuid = short_uuid(0x2D58);

/// Firmware update control (`0x2D60`) — read, write, notify.
///
/// Takes 2-byte big-endian commands (see [`firmware`]) and notifies a 4-byte
/// little-endian count of accepted chunks. The device stores that count into
/// the attribute before notifying it, so the same number can be read without
/// subscribing — see [`crate::VitalHandle::read_firmware_chunks_accepted`].
pub const FIRMWARE_CONTROL_CHARACTERISTIC: Uuid = short_uuid(0x2D60);

/// Firmware update payload (`0x2D61`) — write, write-without-response.
pub const FIRMWARE_PAYLOAD_CHARACTERISTIC: Uuid = short_uuid(0x2D61);

/// The value the device leaves in several attributes until something writes
/// them.
///
/// The GATT table initialises the mode, rename, control and payload attributes
/// to this 4-byte placeholder. It matters because those attributes are
/// *readable*: a read before the first write returns this rather than failing,
/// and as a little-endian `u32` it would pass for a chunk count of 1,144,201,745.
/// Callers that read an attribute back should treat it as "not set since boot" —
/// which is what [`crate::VitalHandle::read_mode`] and
/// [`crate::VitalHandle::read_firmware_chunks_accepted`] do.
pub const UNWRITTEN_ATTRIBUTE_VALUE: [u8; 4] = [0x11, 0x22, 0x33, 0x44];

// ── Sensor mode ───────────────────────────────────────────────────────────────

/// What the ADS1299 front end is doing.
///
/// Written as a 2-byte big-endian command to [`SENSOR_MODE_CHARACTERISTIC`].
/// The device has three EEG electrodes; the per-channel variants exist because
/// the device exposes them, and are useful for isolating one electrode.
///
/// Writing a mode is **not** on its own enough to make data flow: the device
/// also requires the EEG characteristic's CCCD to be subscribed, which is a
/// separate code path guarding the notify. [`crate::VitalClient`] subscribes
/// first and writes the mode second, which is the order that works.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SensorMode {
    /// EEG on all three channels (mask `0b111`). The default.
    #[default]
    EegAllChannels,
    /// EEG on channel 1 only (mask `0b001`).
    EegChannel1,
    /// EEG on channel 2 only (mask `0b010`).
    EegChannel2,
    /// EEG on channel 3 only (mask `0b100`).
    EegChannel3,
    /// Electrode impedance via the ADS1299's AC lead-off detection.
    ///
    /// The device programs `LOFF = 0x06` (12 nA excitation at f_DR/4 ≈
    /// 62.5 Hz), `CH1–3SET = 0x67` (powered on, **gain 24**, MUX = `BIAS_DRN`)
    /// and `LOFF_SENSP = 0x07` with `LOFF_SENSN = 0x00`.
    ///
    /// Two consequences of that, both of which matter:
    ///
    /// - Current is injected into the **positive** inputs only, and each
    ///   channel's negative input is routed to the bias drive. So what is
    ///   measured is each positive electrode against the bias reference — not a
    ///   differential pair.
    /// - With `LOFF_SENSN = 0`, negative-input lead-off detection is off, so
    ///   `LOFF_STATN` stays zero. Only
    ///   [`LeadOffStatus::positive_off`](crate::types::LeadOffStatus::positive_off)
    ///   carries information.
    ///
    /// Results arrive on [`EEG_CHARACTERISTIC`] in the same 20-byte framing as
    /// EEG; the client reports them as
    /// [`VitalEvent::Impedance`](crate::types::VitalEvent::Impedance). Use
    /// [`crate::impedance::ImpedanceEstimator`] to turn the alternating samples
    /// into one number per electrode.
    Impedance,
    /// Front end off (mask `0b000`). No samples are produced.
    Off,
}

impl SensorMode {
    /// The 2-byte command value, host-order.
    pub const fn command(self) -> u16 {
        match self {
            Self::EegAllChannels => 0x0001,
            Self::EegChannel1 => 0x0002,
            Self::EegChannel2 => 0x0003,
            Self::EegChannel3 => 0x0004,
            Self::Impedance => 0x0005,
            // The device treats anything >= 6 as "disable"; 6 is the lowest
            // such value, so it is the one sent.
            Self::Off => 0x0006,
        }
    }

    /// The command as it goes on the wire — big-endian.
    ///
    /// ```
    /// # use vital::protocol::SensorMode;
    /// assert_eq!(SensorMode::EegAllChannels.encode(), [0x00, 0x01]);
    /// assert_eq!(SensorMode::Impedance.encode(), [0x00, 0x05]);
    /// ```
    pub const fn encode(self) -> [u8; 2] {
        self.command().to_be_bytes()
    }

    /// The ADS1299 channel-enable mask this mode programs.
    ///
    /// [`SensorMode::Impedance`] enables all three channels but in lead-off
    /// mode, so it reports `0b111` as well.
    pub const fn channel_mask(self) -> u8 {
        match self {
            Self::EegAllChannels | Self::Impedance => 0b111,
            Self::EegChannel1 => 0b001,
            Self::EegChannel2 => 0b010,
            Self::EegChannel3 => 0b100,
            Self::Off => 0b000,
        }
    }

    /// `true` when samples on [`EEG_CHARACTERISTIC`] are impedance readings
    /// rather than EEG.
    pub const fn is_impedance(self) -> bool {
        matches!(self, Self::Impedance)
    }

    /// How many channels this mode streams.
    ///
    /// ```
    /// # use vital::protocol::SensorMode;
    /// assert_eq!(SensorMode::EegAllChannels.channel_count(), 3);
    /// assert_eq!(SensorMode::EegChannel2.channel_count(), 1);
    /// assert_eq!(SensorMode::Off.channel_count(), 0);
    /// ```
    pub const fn channel_count(self) -> usize {
        self.channel_mask().count_ones() as usize
    }

    /// The channel indices this mode streams, ascending.
    ///
    /// ```
    /// # use vital::protocol::SensorMode;
    /// let channels: Vec<usize> = SensorMode::EegChannel3.channels().collect();
    /// assert_eq!(channels, vec![2]);
    /// ```
    pub fn channels(self) -> impl Iterator<Item = usize> {
        let mask = self.channel_mask();
        (0..EEG_CHANNELS).filter(move |channel| mask & (1 << channel) != 0)
    }

    /// Interpret a raw command value exactly as the device's dispatch does.
    ///
    /// Mirrors the handler's own branch order, including the two quirks worth
    /// knowing about: `0x0000` and `0x0001` both mean "all channels", and every
    /// value at or above `0x0006` means "off" rather than being rejected.
    pub const fn from_command(command: u16) -> Self {
        match command {
            0x0000 | 0x0001 => Self::EegAllChannels,
            0x0002 => Self::EegChannel1,
            0x0003 => Self::EegChannel2,
            0x0004 => Self::EegChannel3,
            0x0005 => Self::Impedance,
            _ => Self::Off,
        }
    }
}

impl SensorMode {
    /// Every mode, in command order. Useful for iterating or for a CLI's help.
    pub const ALL: [SensorMode; 6] = [
        SensorMode::EegAllChannels,
        SensorMode::EegChannel1,
        SensorMode::EegChannel2,
        SensorMode::EegChannel3,
        SensorMode::Impedance,
        SensorMode::Off,
    ];

    /// The short name [`FromStr`](core::str::FromStr) accepts and
    /// [`Display`](core::fmt::Display) produces.
    pub const fn name(self) -> &'static str {
        match self {
            Self::EegAllChannels => "eeg",
            Self::EegChannel1 => "ch1",
            Self::EegChannel2 => "ch2",
            Self::EegChannel3 => "ch3",
            Self::Impedance => "impedance",
            Self::Off => "off",
        }
    }
}

impl core::fmt::Display for SensorMode {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.name())
    }
}

/// The error from [`SensorMode`]'s [`FromStr`](core::str::FromStr)
/// implementation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseSensorModeError(String);

impl core::fmt::Display for ParseSensorModeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "unknown sensor mode {:?}; expected one of ", self.0)?;
        for (i, mode) in SensorMode::ALL.iter().enumerate() {
            if i > 0 {
                f.write_str(", ")?;
            }
            f.write_str(mode.name())?;
        }
        Ok(())
    }
}

impl std::error::Error for ParseSensorModeError {}

impl core::str::FromStr for SensorMode {
    type Err = ParseSensorModeError;

    /// Parse a mode name, case-insensitively.
    ///
    /// `"all"` and `"imp"` are accepted as aliases, because they are what people
    /// type.
    ///
    /// ```
    /// # use vital::protocol::SensorMode;
    /// assert_eq!("impedance".parse(), Ok(SensorMode::Impedance));
    /// assert_eq!("IMP".parse(), Ok(SensorMode::Impedance));
    /// assert_eq!("ch2".parse(), Ok(SensorMode::EegChannel2));
    /// assert!("nonsense".parse::<SensorMode>().is_err());
    /// ```
    fn from_str(text: &str) -> Result<Self, Self::Err> {
        match text.to_ascii_lowercase().as_str() {
            "eeg" | "all" => Ok(Self::EegAllChannels),
            "ch1" => Ok(Self::EegChannel1),
            "ch2" => Ok(Self::EegChannel2),
            "ch3" => Ok(Self::EegChannel3),
            "impedance" | "imp" => Ok(Self::Impedance),
            "off" => Ok(Self::Off),
            _ => Err(ParseSensorModeError(text.to_owned())),
        }
    }
}

// ── Device name ───────────────────────────────────────────────────────────────

/// Longest name [`crate::VitalHandle::rename`] will send.
///
/// The device copies the written name into an 18-byte buffer, so 17 bytes plus
/// a NUL is what fits. It does log an `Invalid Name length` error — but the branch
/// guarding it tests the write's *handle*, which is never zero, so that check
/// never fires. Treat the device as doing no length validation at all and do not
/// write past this: the limit is what stands between a long name and a buffer
/// overrun on the device.
pub const MAX_DEVICE_NAME_LEN: usize = 17;

// ── Sampling constants ────────────────────────────────────────────────────────

/// EEG sample rate in Hz.
///
/// The device programs ADS1299 `CONFIG1 = 0x96`, whose `DR[2:0] = 110`
/// selects f_MOD/4096 = 250 SPS.
pub const EEG_FREQUENCY: f64 = 250.0;

/// Number of EEG electrodes.
pub const EEG_CHANNELS: usize = 3;

/// EEG samples per channel in one notification.
pub const EEG_SAMPLES_PER_PACKET: usize = 2;

/// Length of the status header at the front of an EEG notification.
pub const EEG_HEADER_LEN: usize = 2;

/// Total length of an EEG notification: 2-byte header + 2 samples × 3 channels
/// × 3 bytes.
pub const EEG_PACKET_LEN: usize = EEG_HEADER_LEN + EEG_SAMPLES_PER_PACKET * EEG_CHANNELS * 3;

/// Milliseconds between consecutive EEG samples.
///
/// The reciprocal of [`EEG_FREQUENCY`], pre-computed because a CSV writer or a
/// plot axis wants it and recomputing it at each call site invites a typo.
pub const EEG_SAMPLE_INTERVAL_MS: f64 = 1000.0 / EEG_FREQUENCY;

/// Milliseconds between consecutive EEG notifications.
///
/// Each one carries [`EEG_SAMPLES_PER_PACKET`] samples, so this is 8 ms.
pub const EEG_PACKET_INTERVAL_MS: f64 = EEG_SAMPLE_INTERVAL_MS * EEG_SAMPLES_PER_PACKET as f64;

/// Microvolts per raw LSB for EEG samples.
///
/// The device enables channels with `CH*SET = 0x00`, which is **gain 1**.
/// With the ADS1299's 4.5 V internal reference the full-scale span is
/// ±VREF/gain over 24 bits, so one LSB is `(2 × 4.5 / 1) / 2^24` volts.
///
/// Gain 1 is unusually low for scalp EEG, so expect small integer magnitudes.
pub const EEG_SCALE_UV: f64 = 0.536_441_802_978_515_6;

/// Length of an accelerometer notification: 3 × big-endian `i16`.
pub const ACCELEROMETER_PACKET_LEN: usize = 6;

/// Right-shift that turns a raw accelerometer `i16` into its sample count.
///
/// The LIS3DH left-justifies its data in a 16-bit register, and the device
/// shifts down by 4 before scaling — i.e. it treats the reading as 12-bit.
pub const ACCELEROMETER_COUNT_SHIFT: u32 = 4;

/// Accelerometer scale in g per count, as **the device itself computes it**.
///
/// The device holds the LIS3DH high-resolution sensitivity table
/// `[1, 2, 4, 12]` mg and indexes it with a range selector that lives in `.bss`
/// and is only ever *read* — nothing writes it — so the index is always 0 and
/// the scale is always 1 mg/count.
///
/// # This disagrees with the chip's own configuration
///
/// The device's `set_range` writes `CTRL_REG4 = 0x11` unconditionally, which is
/// FS = ±4 g and HR = 0 (normal, 10-bit mode) — not the ±2 g high-resolution
/// setting that 1 mg/count corresponds to. By the datasheet the configured mode
/// is 2 mg per 12-bit count, so this constant reads low by a factor of two.
///
/// It is kept as the device has it, and because
/// the discrepancy is the device's to resolve. Double it if you want
/// datasheet-correct g.
pub const ACCELEROMETER_SCALE_G: f64 = 0.001;

/// Length of a battery notification.
pub const BATTERY_PACKET_LEN: usize = 7;

// ── Impedance measurement ─────────────────────────────────────────────────────

/// The ADS1299's internal reference voltage, in volts.
pub const ADS1299_VREF_V: f64 = 4.5;

/// Channel gain the device programs for impedance mode (`CH*SET = 0x67`).
pub const IMPEDANCE_GAIN: f64 = 24.0;

/// Lead-off excitation current in nanoamps (`LOFF = 0x06`, `ILEAD_OFF = 01`).
pub const IMPEDANCE_EXCITATION_NA: f64 = 12.0;

/// Frequency of the lead-off excitation, in Hz (`FLEAD_OFF = 10`, f_DR/4).
pub const IMPEDANCE_EXCITATION_HZ: f64 = EEG_FREQUENCY / 4.0;

/// Samples per excitation cycle.
///
/// Exactly 4, because the excitation is at f_DR/4 and sampling is at f_DR — which
/// is what makes a clean quadrature demodulation possible with no windowing. See
/// [`crate::impedance::ImpedanceEstimator`].
pub const IMPEDANCE_SAMPLES_PER_CYCLE: usize = 4;

/// Ohms per unit of demodulated code amplitude, from Ohm's law.
///
/// At gain 24 one LSB is `(2 × VREF / gain) / 2^24` = 22.35 nV, and the
/// excitation is 12 nA, so `Z = amplitude × 22.35 nV / 12 nA`.
///
/// This is derived from the datasheet and the register values the device
/// writes, **not measured** — the device performs no conversion of its own, so
/// there is nothing to check it against short of a known resistor across the
/// inputs. Treat the absolute scale as provisional and the relative values as
/// sound; [`crate::impedance::ImpedanceEstimate::amplitude_codes`] is the
/// unscaled figure if you would rather calibrate it yourself.
pub const IMPEDANCE_OHMS_PER_CODE: f64 =
    (2.0 * ADS1299_VREF_V / IMPEDANCE_GAIN / 16_777_216.0) * 1e9 / IMPEDANCE_EXCITATION_NA;

// ── Battery ───────────────────────────────────────────────────────────────────

/// Battery millivolts the device treats as 0 %.
pub const BATTERY_ZERO_PERCENT_MV: u16 = 3150;

/// Millivolts per percentage point in the device's state-of-charge curve.
///
/// It is a straight line: `percent = (millivolts - 3150) / 10`.
pub const BATTERY_MV_PER_PERCENT: u16 = 10;

/// Largest value [`crate::types::BatteryReading::percent`] can report.
///
/// **Not 100.** The device clamps to 100 only once the voltage is at or above
/// 4200 mV, but its linear curve already reaches 100 at 4150 mV — so between
/// 4150 and 4199 mV the unclamped arithmetic runs on and reports 101 to 104.
///
/// Anything dividing the percentage by 100 for a progress bar needs its own
/// clamp; see [`crate::types::BatteryReading::percent_clamped`].
pub const BATTERY_MAX_REPORTED_PERCENT: u8 = 104;

/// Battery millivolts at or above which the device clamps to 100 %.
pub const BATTERY_FULL_MV: u16 = 4200;

/// Channel labels, in wire order.
///
/// Named by index on purpose. The device calls them "sens 1/2/3" and nothing
/// in it maps a channel to a scalp position, so a 10-20 name here would be a
/// wrong answer that looks like a right one.
pub const EEG_CHANNEL_NAMES: [&str; EEG_CHANNELS] = ["ch1", "ch2", "ch3"];

/// The label for `channel`, or `"?"` if out of range.
pub fn eeg_channel_name(channel: usize) -> &'static str {
    EEG_CHANNEL_NAMES.get(channel).copied().unwrap_or("?")
}

// ── Firmware version ──────────────────────────────────────────────────────────

/// The device's firmware version and build date.
///
/// The characteristic returns both concatenated with no separator: the version
/// string and the build date are copied into one fixed-width buffer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FirmwareVersion {
    /// The version, e.g. `"4.3.8"`.
    pub version: String,
    /// The compile date in C `__DATE__` form, e.g. `"May  9 2022"`.
    pub built: String,
    /// The response exactly as read, trailing NULs trimmed.
    pub raw: String,
}

/// Split a firmware-version read into its two halves.
///
/// A C `__DATE__` is always exactly 11 characters (`"Mmm dd yyyy"`, with the
/// day space-padded), so the split point is 11 from the end — which is the only
/// thing that can separate them, there being no delimiter.
///
/// A response shorter than that is returned whole as `version`, with an empty
/// `built`, rather than being rejected.
pub fn parse_firmware_version(bytes: &[u8]) -> FirmwareVersion {
    const DATE_LEN: usize = 11;

    let raw = String::from_utf8_lossy(bytes)
        .trim_end_matches('\0')
        .trim()
        .to_owned();

    match raw.char_indices().nth_back(DATE_LEN - 1) {
        Some((split, _)) if raw.len() > DATE_LEN => FirmwareVersion {
            version: raw[..split].to_owned(),
            built: raw[split..].to_owned(),
            raw,
        },
        _ => FirmwareVersion {
            version: raw.clone(),
            built: String::new(),
            raw,
        },
    }
}

// ── Firmware update wire format ───────────────────────────────────────────────

/// The over-the-air update protocol, as the device speaks it.
///
/// Encoders and decoders only. There is deliberately no "flash this file"
/// helper here: an interrupted or mis-framed update leaves the device booting
/// from a half-written partition, and that is not a thing to make reachable by
/// accident. The pieces are complete enough to drive an update deliberately.
///
/// The sequence is: write [`OTA_BEGIN`](firmware::OTA_BEGIN) to
/// [`FIRMWARE_CONTROL_CHARACTERISTIC`], write chunks from [`encode_chunk`](firmware::encode_chunk) to
/// [`FIRMWARE_PAYLOAD_CHARACTERISTIC`] with `index` starting at **1** and
/// incrementing by exactly 1, then write [`OTA_END`](firmware::OTA_END) — at which point the
/// device sets its boot partition and reboots.
///
/// After every accepted chunk the device notifies the control characteristic
/// with the running count; [`decode_progress`](firmware::decode_progress) reads it. A chunk whose index is
/// not `previous + 1` is rejected and the count does not advance.
pub mod firmware {
    /// Open the update: `esp_ota_begin` on the inactive partition.
    pub const OTA_BEGIN: u16 = 0x0001;
    /// Close the update, set the boot partition, and reboot.
    pub const OTA_END: u16 = 0x0002;
    /// Ask for the accepted-chunk count to be notified back.
    pub const OTA_QUERY_PROGRESS: u16 = 0x0003;

    /// Encode a control command — 2 bytes, big-endian.
    pub const fn encode_control(command: u16) -> [u8; 2] {
        command.to_be_bytes()
    }

    /// Frame one payload chunk: a 4-byte **little-endian** index followed by
    /// the device bytes.
    ///
    /// Note the endianness flip relative to the control characteristic. The
    /// first chunk must carry index 1.
    pub fn encode_chunk(index: u32, data: &[u8]) -> Vec<u8> {
        let mut framed = Vec::with_capacity(4 + data.len());
        framed.extend_from_slice(&index.to_le_bytes());
        framed.extend_from_slice(data);
        framed
    }

    /// Read a progress notification: the count of chunks accepted so far,
    /// 4 bytes little-endian.
    pub fn decode_progress(bytes: &[u8]) -> Option<u32> {
        let four: [u8; 4] = bytes.get(..4)?.try_into().ok()?;
        Some(u32::from_le_bytes(four))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn short_uuid_expands_to_the_sig_base() {
        assert_eq!(
            VITAL_SERVICE_UUID.to_string(),
            "00002d0d-0000-1000-8000-00805f9b34fb"
        );
        assert_eq!(
            EEG_CHARACTERISTIC.to_string(),
            "00002d37-0000-1000-8000-00805f9b34fb"
        );
    }

    #[test]
    fn mode_commands_are_big_endian() {
        assert_eq!(SensorMode::EegAllChannels.encode(), [0x00, 0x01]);
        assert_eq!(SensorMode::Impedance.encode(), [0x00, 0x05]);
        assert_eq!(SensorMode::Off.encode(), [0x00, 0x06]);
    }

    #[test]
    fn from_command_matches_the_firmware_dispatch() {
        // 0 and 1 both mean "all channels".
        assert_eq!(SensorMode::from_command(0), SensorMode::EegAllChannels);
        assert_eq!(SensorMode::from_command(1), SensorMode::EegAllChannels);
        assert_eq!(SensorMode::from_command(4), SensorMode::EegChannel3);
        // Everything from 6 up is "off", not an error.
        assert_eq!(SensorMode::from_command(6), SensorMode::Off);
        assert_eq!(SensorMode::from_command(0xFFFF), SensorMode::Off);
        // Every mode round-trips through its own command value.
        for mode in [
            SensorMode::EegAllChannels,
            SensorMode::EegChannel1,
            SensorMode::EegChannel2,
            SensorMode::EegChannel3,
            SensorMode::Impedance,
            SensorMode::Off,
        ] {
            assert_eq!(SensorMode::from_command(mode.command()), mode);
        }
    }

    #[test]
    fn packet_length_is_the_twenty_bytes_the_device_sends() {
        assert_eq!(EEG_PACKET_LEN, 20);
    }

    #[test]
    fn firmware_version_splits_off_the_build_date() {
        let v = parse_firmware_version(b"4.3.8May  9 2022");
        assert_eq!(v.version, "4.3.8");
        assert_eq!(v.built, "May  9 2022");

        // A six-character version, which shifts the split point.
        let v = parse_firmware_version(b"4.3.12Jan  4 2024");
        assert_eq!(v.version, "4.3.12");
        assert_eq!(v.built, "Jan  4 2024");

        // Too short to contain a date: kept whole rather than mangled.
        let v = parse_firmware_version(b"4.3.8");
        assert_eq!(v.version, "4.3.8");
        assert_eq!(v.built, "");
    }

    #[test]
    fn the_placeholder_is_not_mistakable_for_real_data() {
        // It is 4 bytes, so it can never be mistaken for a 2-byte mode command.
        assert_ne!(UNWRITTEN_ATTRIBUTE_VALUE.len(), 2);
        // As a chunk count it would be absurd, which is the whole reason
        // `read_firmware_chunks_accepted` checks for it rather than decoding.
        assert_eq!(
            firmware::decode_progress(&UNWRITTEN_ATTRIBUTE_VALUE),
            Some(0x4433_2211)
        );
    }

    #[test]
    fn chunk_index_is_little_endian_and_prefixes_the_data() {
        assert_eq!(
            firmware::encode_chunk(1, &[0xAA, 0xBB]),
            vec![0x01, 0x00, 0x00, 0x00, 0xAA, 0xBB]
        );
        assert_eq!(
            firmware::decode_progress(&[0x01, 0x00, 0x00, 0x00]),
            Some(1)
        );
        assert_eq!(firmware::decode_progress(&[0x01]), None);
    }
}
