//! Turning impedance notifications into one number per electrode.
//!
//! In [`SensorMode::Impedance`](crate::protocol::SensorMode::Impedance) the
//! front end injects a small alternating current into each positive electrode and
//! the channel data carries the resulting voltage. A single sample is therefore a
//! point on a sine wave: it tells you almost nothing, and consecutive samples
//! swing between large positive and large negative values even on a perfectly
//! attached electrode. The quantity you want is the wave's **amplitude**.
//!
//! Two properties of this particular device make that easy and exact:
//!
//! - The excitation is at `f_DR/4` and sampling is at `f_DR`, so there are
//!   exactly **4 samples per cycle**, 90° apart. No window, no interpolation, no
//!   leakage.
//! - A 4-sample quadrature demodulation rejects DC perfectly, which matters
//!   because the electrode offset is far larger than the excitation response.
//!
//! For samples `s0..s3` at 90° steps, writing `s[n] = A·cos(πn/2 + φ)`:
//!
//! ```text
//! I = (s0 − s2) / 2 =  A·cos φ
//! Q = (s1 − s3) / 2 = −A·sin φ
//! A = hypot(I, Q)
//! ```
//!
//! The phase `φ` falls out, so no synchronisation with the device is needed.

use crate::protocol::{
    EEG_CHANNELS, EEG_SAMPLES_PER_PACKET, IMPEDANCE_OHMS_PER_CODE, IMPEDANCE_SAMPLES_PER_CYCLE,
};
use crate::types::{ImpedanceFrame, LeadOffStatus};

/// One impedance measurement per channel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImpedanceEstimate {
    /// Wall-clock milliseconds of the most recent frame that fed this estimate.
    pub timestamp: f64,
    /// Demodulated amplitude in ADC codes, per channel.
    ///
    /// `None` where the channel was not streaming for the whole window. This is
    /// the raw measurement: unscaled, and the thing to calibrate against a known
    /// resistor if you want trustworthy absolute ohms.
    pub amplitude_codes: [Option<f64>; EEG_CHANNELS],
    /// Electrode contact flags from the last frame in the window.
    ///
    /// The front end's own yes/no, needing no calibration — a useful cross-check
    /// on the computed magnitude.
    pub lead_off: LeadOffStatus,
    /// How many excitation cycles each amplitude was averaged over.
    pub cycles: usize,
}

impl ImpedanceEstimate {
    /// The estimate converted to ohms via [`IMPEDANCE_OHMS_PER_CODE`].
    ///
    /// Derived from the datasheet and the device's configuration, not
    /// measured — read that constant's documentation before quoting the number.
    /// Relative comparisons between electrodes are on much firmer ground than the
    /// absolute scale.
    pub fn ohms(&self) -> [Option<f64>; EEG_CHANNELS] {
        let mut out = [None; EEG_CHANNELS];
        for (slot, amplitude) in out.iter_mut().zip(self.amplitude_codes) {
            *slot = amplitude.map(|a| a * IMPEDANCE_OHMS_PER_CODE);
        }
        out
    }

    /// The ohms for one channel, or `None` if it was not measured.
    ///
    /// Equivalent to indexing [`ImpedanceEstimate::ohms`], but without building
    /// the whole array, and safe for a channel index out of range.
    pub fn ohms_for(&self, channel: usize) -> Option<f64> {
        let amplitude = (*self.amplitude_codes.get(channel)?)?;
        Some(amplitude * IMPEDANCE_OHMS_PER_CODE)
    }

    /// How many channels this estimate covers.
    pub fn channel_count(&self) -> usize {
        self.amplitude_codes.iter().flatten().count()
    }

    /// The worst (largest) amplitude across channels, and which channel it was.
    ///
    /// What a "check the fit" display wants: the electrode most in need of
    /// attention.
    pub fn worst(&self) -> Option<(usize, f64)> {
        self.amplitude_codes
            .iter()
            .enumerate()
            .filter_map(|(i, a)| a.map(|a| (i, a)))
            .max_by(|(_, a), (_, b)| a.total_cmp(b))
    }
}

/// Accumulates impedance frames and emits one [`ImpedanceEstimate`] per window.
///
/// Feed it every [`ImpedanceFrame`]; it returns `Some` once it has a whole number
/// of excitation cycles. Each frame carries two samples, so one cycle is two
/// frames — an estimate every 16 ms at the default of one cycle.
///
/// ```
/// use vital::impedance::ImpedanceEstimator;
/// use vital::types::{ImpedanceFrame, LeadOffStatus};
///
/// // Averaging over 16 cycles ≈ one estimate every 256 ms: steadier, and still
/// // fast enough to watch someone adjust the headphones.
/// let mut estimator = ImpedanceEstimator::with_averaging(16);
///
/// # let frame = ImpedanceFrame {
/// #     timestamp: 0.0,
/// #     lead_off: LeadOffStatus::default(),
/// #     channels: [Some([0, 0]), None, None],
/// # };
/// if let Some(estimate) = estimator.push(&frame) {
///     println!("{:?}", estimate.ohms());
/// }
/// ```
#[derive(Debug, Clone)]
pub struct ImpedanceEstimator {
    /// Rolling 4-sample window per channel, oldest first.
    window: [[i32; IMPEDANCE_SAMPLES_PER_CYCLE]; EEG_CHANNELS],
    /// Samples pushed into each channel's window since the estimator started.
    filled: [usize; EEG_CHANNELS],
    /// Per-channel sum of completed cycle amplitudes, and how many there were.
    sums: [f64; EEG_CHANNELS],
    counts: [usize; EEG_CHANNELS],
    /// Samples seen since the last emitted estimate, used to align cycles.
    phase: usize,
    cycles_per_estimate: usize,
    last_timestamp: f64,
    last_lead_off: LeadOffStatus,
}

impl Default for ImpedanceEstimator {
    fn default() -> Self {
        Self::new()
    }
}

impl ImpedanceEstimator {
    /// One estimate per excitation cycle — the fastest the device allows.
    pub fn new() -> Self {
        Self::with_averaging(1)
    }

    /// Average each estimate over `cycles` excitation cycles.
    ///
    /// Amplitude noise falls as roughly `1/sqrt(cycles)`. `cycles` is clamped to
    /// at least 1, so a zero does not mean "never emit".
    pub fn with_averaging(cycles: usize) -> Self {
        Self {
            window: [[0; IMPEDANCE_SAMPLES_PER_CYCLE]; EEG_CHANNELS],
            filled: [0; EEG_CHANNELS],
            sums: [0.0; EEG_CHANNELS],
            counts: [0; EEG_CHANNELS],
            phase: 0,
            cycles_per_estimate: cycles.max(1),
            last_timestamp: 0.0,
            last_lead_off: LeadOffStatus::default(),
        }
    }

    /// Feed one frame. Returns an estimate when a window completes.
    pub fn push(&mut self, frame: &ImpedanceFrame) -> Option<ImpedanceEstimate> {
        self.last_timestamp = frame.timestamp;
        self.last_lead_off = frame.lead_off;

        let mut emit = None;
        for sample_index in 0..EEG_SAMPLES_PER_PACKET {
            for channel in 0..EEG_CHANNELS {
                match frame.channels[channel] {
                    Some(codes) => {
                        // Shift the window and append. Four elements, so a
                        // rotate is cheaper to read than an index dance.
                        self.window[channel].rotate_left(1);
                        self.window[channel][IMPEDANCE_SAMPLES_PER_CYCLE - 1] = codes[sample_index];
                        self.filled[channel] += 1;
                    }
                    None => {
                        // A channel that stops streaming mid-window must not
                        // contribute a half-window amplitude.
                        self.filled[channel] = 0;
                    }
                }
            }

            self.phase += 1;
            // A cycle boundary: every channel with a full window contributes one
            // amplitude. Aligning on a multiple of the cycle length means
            // consecutive cycles do not overlap, so averaging really does average
            // independent measurements.
            if self.phase.is_multiple_of(IMPEDANCE_SAMPLES_PER_CYCLE) {
                for channel in 0..EEG_CHANNELS {
                    if self.filled[channel] >= IMPEDANCE_SAMPLES_PER_CYCLE {
                        self.sums[channel] += amplitude(&self.window[channel]);
                        self.counts[channel] += 1;
                    }
                }
            }

            if self.phase >= self.cycles_per_estimate * IMPEDANCE_SAMPLES_PER_CYCLE {
                emit = Some(self.take());
            }
        }
        emit
    }

    /// Emit the accumulated estimate and reset the window.
    fn take(&mut self) -> ImpedanceEstimate {
        let mut amplitude_codes = [None; EEG_CHANNELS];
        for (slot, (sum, count)) in amplitude_codes
            .iter_mut()
            .zip(self.sums.iter().zip(self.counts))
        {
            if count > 0 {
                *slot = Some(sum / count as f64);
            }
        }
        let estimate = ImpedanceEstimate {
            timestamp: self.last_timestamp,
            amplitude_codes,
            lead_off: self.last_lead_off,
            cycles: self.cycles_per_estimate,
        };
        self.sums = [0.0; EEG_CHANNELS];
        self.counts = [0; EEG_CHANNELS];
        self.phase = 0;
        estimate
    }

    /// Forget everything accumulated so far.
    ///
    /// Call this after a mode change or a reconnect, so samples from before are
    /// not averaged with samples from after.
    pub fn reset(&mut self) {
        *self = Self::with_averaging(self.cycles_per_estimate);
    }
}

/// Quadrature amplitude of four samples spaced 90° apart.
///
/// Both differences cancel any constant offset exactly, which is the point: the
/// electrode's DC offset dwarfs the excitation response.
fn amplitude(window: &[i32; IMPEDANCE_SAMPLES_PER_CYCLE]) -> f64 {
    let i = (window[0] as f64 - window[2] as f64) / 2.0;
    let q = (window[1] as f64 - window[3] as f64) / 2.0;
    i.hypot(q)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Frames carrying a synthetic sine of known amplitude, phase and offset on
    /// channel 0.
    fn frames(amplitude: f64, phase: f64, offset: f64, count: usize) -> Vec<ImpedanceFrame> {
        let mut sample = 0usize;
        (0..count)
            .map(|_| {
                let mut codes = [0i32; EEG_SAMPLES_PER_PACKET];
                for slot in codes.iter_mut() {
                    let t = PI / 2.0 * sample as f64 + phase;
                    *slot = (amplitude * t.cos() + offset).round() as i32;
                    sample += 1;
                }
                ImpedanceFrame {
                    timestamp: sample as f64,
                    lead_off: LeadOffStatus::default(),
                    channels: [Some(codes), None, None],
                }
            })
            .collect()
    }

    #[test]
    fn recovers_amplitude_regardless_of_phase_and_offset() {
        // A large DC offset is the realistic case and must cancel exactly.
        for phase in [0.0, 0.3, 1.0, 2.5, 6.0] {
            for offset in [0.0, 50_000.0, -120_000.0] {
                let mut est = ImpedanceEstimator::with_averaging(4);
                let mut got = None;
                for frame in frames(10_000.0, phase, offset, 16) {
                    if let Some(e) = est.push(&frame) {
                        got = Some(e);
                        break;
                    }
                }
                let e = got.expect("an estimate within 16 frames");
                let a = e.amplitude_codes[0].expect("channel 0 streamed throughout");
                // Rounding to integer codes is the only error source here.
                assert!(
                    (a - 10_000.0).abs() < 5.0,
                    "phase {phase} offset {offset}: got {a}"
                );
                assert_eq!(e.cycles, 4);
            }
        }
    }

    #[test]
    fn ohms_scales_the_amplitude() {
        let mut est = ImpedanceEstimator::new();
        let mut got = None;
        for frame in frames(1000.0, 0.0, 0.0, 4) {
            if let Some(e) = est.push(&frame) {
                got = Some(e);
                break;
            }
        }
        let e = got.unwrap();
        let amplitude = e.amplitude_codes[0].unwrap();
        let ohms = e.ohms()[0].unwrap();
        assert!((ohms - amplitude * IMPEDANCE_OHMS_PER_CODE).abs() < 1e-9);
        // Sanity on the derived constant: ~1.86 Ω per code.
        assert!((IMPEDANCE_OHMS_PER_CODE - 1.862_645).abs() < 1e-5);
    }

    #[test]
    fn channels_that_are_not_streaming_stay_none() {
        let mut est = ImpedanceEstimator::new();
        let mut got = None;
        for frame in frames(1000.0, 0.0, 0.0, 4) {
            if let Some(e) = est.push(&frame) {
                got = Some(e);
                break;
            }
        }
        let e = got.unwrap();
        assert!(e.amplitude_codes[0].is_some());
        assert!(e.amplitude_codes[1].is_none());
        assert!(e.amplitude_codes[2].is_none());
        assert_eq!(e.worst().map(|(i, _)| i), Some(0));
    }

    #[test]
    fn a_channel_dropping_out_does_not_yield_a_partial_amplitude() {
        let mut est = ImpedanceEstimator::new();
        // One frame of data, then a frame with the channel gone: the window never
        // completes, so nothing is reported for it.
        let mut iter = frames(1000.0, 0.0, 0.0, 2).into_iter();
        est.push(&iter.next().unwrap());
        let gap = ImpedanceFrame {
            timestamp: 9.0,
            lead_off: LeadOffStatus::default(),
            channels: [None, None, None],
        };
        let e = est.push(&gap).expect("the window still closes on schedule");
        assert!(e.amplitude_codes[0].is_none());
        assert!(e.worst().is_none());
    }

    #[test]
    fn reset_clears_accumulated_state() {
        let mut est = ImpedanceEstimator::with_averaging(8);
        for frame in frames(1000.0, 0.0, 0.0, 3) {
            est.push(&frame);
        }
        est.reset();
        // After a reset it takes a full window again, so three frames of the
        // eight-cycle window cannot produce anything.
        for frame in frames(1000.0, 0.0, 0.0, 3) {
            assert!(est.push(&frame).is_none());
        }
    }
}
