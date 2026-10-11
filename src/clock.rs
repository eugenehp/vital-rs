//! Timestamp reconstruction for notifications that carry no device clock.

use log::debug;

use crate::protocol::EEG_PACKET_INTERVAL_MS;

/// How far the extrapolated clock may drift from wall time before re-anchoring.
const RESYNC_THRESHOLD_MS: f64 = 1000.0;

/// Reconstructs a timestamp for each notification by counting packets.
///
/// Headsets that embed a sequence counter let you extrapolate timestamps and
/// *correct* them: a gap in the counter tells you packets were lost and by how
/// much. This device embeds no counter — the only non-sample field is the two-byte
/// lead-off header — so the best available is to anchor on the first packet and
/// step forward one packet-duration at a time.
///
/// The failure mode that leaves is loss: a dropped notification is invisible, so
/// the prediction falls behind the wall clock by one step each time it happens.
/// Rather than let that grow without bound, the clock re-anchors once it has
/// drifted past [`RESYNC_THRESHOLD_MS`] — accepting one visible discontinuity
/// instead of an unbounded invisible skew.
///
/// [`PacketClock::next`] takes the current time rather than reading a clock, which
/// is what makes the drift and re-anchor behaviour testable.
#[derive(Debug, Clone)]
pub(crate) struct PacketClock {
    anchor_ms: Option<f64>,
    packets_since_anchor: u64,
    step_ms: f64,
}

impl PacketClock {
    /// A clock stepping `step_ms` per packet.
    pub(crate) fn new(step_ms: f64) -> Self {
        Self {
            anchor_ms: None,
            packets_since_anchor: 0,
            step_ms,
        }
    }

    /// A clock for this device's EEG and impedance streams.
    pub(crate) fn for_eeg() -> Self {
        Self::new(EEG_PACKET_INTERVAL_MS)
    }

    /// The timestamp for a packet arriving at `now_ms`.
    pub(crate) fn next(&mut self, now_ms: f64) -> f64 {
        match self.anchor_ms {
            Some(anchor) => {
                let predicted = anchor + self.packets_since_anchor as f64 * self.step_ms;
                if (now_ms - predicted).abs() > RESYNC_THRESHOLD_MS {
                    debug!(
                        "packet clock drifted {:.0} ms from wall time — re-anchoring",
                        now_ms - predicted
                    );
                    self.anchor_ms = Some(now_ms);
                    self.packets_since_anchor = 1;
                    now_ms
                } else {
                    self.packets_since_anchor += 1;
                    predicted
                }
            }
            None => {
                self.anchor_ms = Some(now_ms);
                self.packets_since_anchor = 1;
                now_ms
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_packet_anchors_to_the_clock() {
        let mut clock = PacketClock::for_eeg();
        assert_eq!(clock.next(5_000.0), 5_000.0);
    }

    #[test]
    fn subsequent_packets_step_uniformly_despite_jitter() {
        let mut clock = PacketClock::for_eeg();
        clock.next(0.0);
        // Arrival times are deliberately uneven; output must not be.
        let stamps: Vec<f64> = [3.0, 19.0, 21.0, 40.0]
            .into_iter()
            .map(|now| clock.next(now))
            .collect();
        assert_eq!(stamps, vec![8.0, 16.0, 24.0, 32.0]);
    }

    #[test]
    fn drift_past_the_threshold_re_anchors() {
        let mut clock = PacketClock::for_eeg();
        clock.next(0.0);
        clock.next(8.0);
        // A two-second gap: far more than the prediction can absorb, so the next
        // timestamp is the wall clock rather than 16 ms.
        let after_gap = clock.next(2_000.0);
        assert_eq!(after_gap, 2_000.0);
        // And it resumes stepping from the new anchor.
        assert_eq!(clock.next(2_008.0), 2_008.0);
    }

    #[test]
    fn drift_within_the_threshold_does_not_re_anchor() {
        let mut clock = PacketClock::for_eeg();
        clock.next(0.0);
        // Half a second late is well inside the threshold: keep extrapolating, so
        // the sample spacing stays exactly 8 ms.
        assert_eq!(clock.next(500.0), 8.0);
        assert_eq!(clock.next(500.0), 16.0);
    }

    #[test]
    fn a_clock_running_ahead_also_re_anchors() {
        let mut clock = PacketClock::for_eeg();
        clock.next(10_000.0);
        // Wall time jumped backwards (NTP step, suspend/resume): the magnitude of
        // the drift is what matters, not its sign.
        assert_eq!(clock.next(5_000.0), 5_000.0);
    }
}
