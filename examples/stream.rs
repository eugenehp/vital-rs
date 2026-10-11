//! The smallest useful program: mean EEG amplitude per channel, once a second.
//!
//! ```text
//! cargo run --example stream
//! ```

use anyhow::Result;
use vital::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::init();

    let (mut events, handle) = VitalClient::new(VitalClientConfig::default())
        .connect()
        .await?;
    handle.start().await?;

    let mut sums = [0.0f64; EEG_CHANNELS];
    let mut counts = [0usize; EEG_CHANNELS];
    let mut frames = 0usize;

    // 250 Hz in frames of 2 samples is 125 frames per second.
    let frames_per_second = (EEG_FREQUENCY / EEG_SAMPLES_PER_PACKET as f64) as usize;

    while let Some(event) = events.recv().await {
        match event {
            VitalEvent::Eeg(frame) => {
                for (channel, samples) in frame.enabled() {
                    sums[channel] += samples.uv.iter().map(|s| s.abs()).sum::<f64>();
                    counts[channel] += samples.uv.len();
                }
                frames += 1;

                if frames >= frames_per_second {
                    for channel in 0..EEG_CHANNELS {
                        if counts[channel] > 0 {
                            println!(
                                "{}: mean |amplitude| {:.2} µV over {} samples",
                                eeg_channel_name(channel),
                                sums[channel] / counts[channel] as f64,
                                counts[channel]
                            );
                        }
                    }
                    sums = [0.0; EEG_CHANNELS];
                    counts = [0; EEG_CHANNELS];
                    frames = 0;
                }
            }
            VitalEvent::Disconnected => break,
            _ => {}
        }
    }
    Ok(())
}
