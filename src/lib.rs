//! # vital
//!
//! Async Rust client for **Vital Neuro** EEG headphones over Bluetooth Low
//! Energy, built on [`webbluetooth`] so the same code runs natively and in a
//! browser.
//!
//! ## The device
//!
//! | | |
//! |---|---|
//! | Link | dual-mode — BLE for EEG, Bluetooth Classic A2DP for audio |
//! | Front end | TI ADS1299, **3 EEG channels at 250 SPS**, gain 1 |
//! | Also on board | LIS3DH accelerometer, battery/charger telemetry |
//! | GATT | one service, `0x2D0D`, with 16-bit assigned-number characteristic UUIDs |
//!
//! This crate speaks only the BLE side. Audio is a separate link; pair the
//! headphones with the OS as normal and the two coexist.
//!
//! ## Quick start
//!
//! ```no_run
//! use vital::prelude::*;
//!
//! #[tokio::main]
//! async fn main() -> anyhow::Result<()> {
//!     let client = VitalClient::new(VitalClientConfig::default());
//!     let (mut rx, handle) = client.connect().await?;
//!
//!     println!("{:?}", handle.firmware_version().await?);
//!     handle.start().await?;
//!
//!     while let Some(event) = rx.recv().await {
//!         match event {
//!             VitalEvent::Eeg(frame) => {
//!                 for (channel, samples) in frame.enabled() {
//!                     println!("{}: {:?} µV", eeg_channel_name(channel), samples.uv);
//!                 }
//!             }
//!             VitalEvent::Disconnected => break,
//!             _ => {}
//!         }
//!     }
//!     Ok(())
//! }
//! ```
//!
//! ## Two things that will bite you
//!
//! **Subscribing is half of starting.** The device arms its sampling task from
//! the EEG characteristic's CCCD handler, *and* gates it on the mode written to
//! [`protocol::SENSOR_MODE_CHARACTERISTIC`]. Neither alone produces data.
//! [`VitalClient::connect`] subscribes and [`VitalHandle::start`] writes the
//! mode, in that order — which is the order that works.
//!
//! **Impedance shares the EEG characteristic.** Selecting
//! [`protocol::SensorMode::Impedance`] does not open a second stream; the same
//! 20-byte notifications start carrying lead-off measurements instead of EEG.
//! The client tracks the mode and emits [`types::VitalEvent::Impedance`]
//! accordingly, so a consumer matching on the event type does the right thing.
//!
//! ## Limits
//!
//! Every notification payload is decoded. One thing is not *calibrated*: the
//! absolute scale from impedance codes to ohms
//! ([`protocol::IMPEDANCE_OHMS_PER_CODE`]) is derived from the datasheet and the
//! device's register writes rather than measured, because the device performs
//! no conversion of its own to check against. Relative comparisons between
//! electrodes are unaffected, and [`types::LeadOffStatus`] answers "is this
//! electrode on?" with no calibration at all.
//!
//! One decoded value needs a caveat rather than a disclaimer: the accelerometer
//! scale is reproduced from the device, whose own constant disagrees with the
//! range it programs into the chip. See [`protocol::ACCELEROMETER_SCALE_G`].
//!
//! ## Module overview
//!
//! | Module | Purpose |
//! |---|---|
//! | [`prelude`] | One-line glob import of everything you normally need |
//! | [`vital_client`] | Scanning, connecting, and the command handle |
//! | [`types`] | Events and data types |
//! | [`protocol`] | UUIDs, sensor commands, sampling constants |
//! | [`parse`] | Byte-level decoders, testable without a device |
//! | [`impedance`] | Turns impedance frames into one number per electrode |

#![deny(missing_docs)]
#![deny(missing_debug_implementations)]
#![deny(rustdoc::broken_intra_doc_links)]

mod clock;

pub mod impedance;
pub mod parse;
pub mod protocol;
pub mod types;
pub mod vital_client;

// Spawning and timers differ between a native runtime and a browser event loop.
// Internal: which one is in use is not something a caller chooses.
mod platform;

pub use vital_client::{VitalClient, VitalClientConfig, VitalDevice, VitalHandle};

/// Convenience re-exports: one glob import covers scan, connect, and consume.
pub mod prelude {
    pub use crate::vital_client::{VitalClient, VitalClientConfig, VitalDevice, VitalHandle};

    pub use crate::types::{
        AccelerometerReading, BatteryReading, ChannelSamples, EegFrame, FirmwareProgress,
        ImpedanceFrame, LeadOffStatus, VitalEvent,
    };

    pub use crate::impedance::{ImpedanceEstimate, ImpedanceEstimator};

    pub use crate::protocol::{
        eeg_channel_name, FirmwareVersion, ParseSensorModeError, SensorMode, EEG_CHANNELS,
        EEG_CHANNEL_NAMES, EEG_FREQUENCY, EEG_PACKET_INTERVAL_MS, EEG_PACKET_LEN,
        EEG_SAMPLES_PER_PACKET, EEG_SAMPLE_INTERVAL_MS, EEG_SCALE_UV, MAX_DEVICE_NAME_LEN,
    };

    pub use crate::parse::EegPacket;
}
