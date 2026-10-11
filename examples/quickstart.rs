//! The README's quick-start example, verbatim.
//!
//! It lives here so `cargo build --examples` keeps the README honest: if the API
//! moves, this stops compiling and the docs get fixed with it.
//!
//! ```text
//! cargo run --example quickstart
//! ```

use vital::prelude::*;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let (mut events, handle) = VitalClient::new(VitalClientConfig::default())
        .connect()
        .await?;

    let version = handle.firmware_version().await?;
    println!("firmware {} (built {})", version.version, version.built);

    handle.start().await?;

    while let Some(event) = events.recv().await {
        match event {
            VitalEvent::Eeg(frame) => {
                for (channel, samples) in frame.enabled() {
                    println!("{}: {:?} µV", eeg_channel_name(channel), samples.uv);
                }
            }
            VitalEvent::Battery(b) => println!("{}% / {} mV", b.percent_clamped(), b.millivolts()),
            VitalEvent::Disconnected => break,
            _ => {}
        }
    }
    Ok(())
}
