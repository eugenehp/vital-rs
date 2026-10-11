//! Headless CLI for Vital Neuro headphones.
//!
//! ```text
//! vital scan                 list every device in range
//! vital info                 firmware version, mode and battery, then exit
//! vital eeg                  stream EEG (the default)
//! vital eeg --csv            the same, as CSV on stdout
//! vital impedance            electrode contact, in ohms
//! vital ch1 | ch2 | ch3      stream one channel
//! ```
//!
//! Mode names are [`SensorMode`]'s own, so `--help` and the parser cannot drift
//! apart from the library.
//!
//! `RUST_LOG=info` adds protocol logging on stderr, which stays out of the way of
//! `--csv` on stdout.

use std::time::Duration;

use anyhow::Result;
use vital::prelude::*;

/// Cycles to average each impedance estimate over.
///
/// 16 cycles is ~256 ms: steady enough to read off a terminal, fast enough to
/// watch someone adjust the headphones.
const IMPEDANCE_AVERAGING_CYCLES: usize = 16;

/// How long `info` waits for a battery notification before giving up.
const BATTERY_WAIT_SECS: u64 = 10;

enum Command {
    Scan,
    Info,
    Stream { mode: SensorMode, csv: bool },
}

fn usage() -> ! {
    let modes: Vec<&str> = SensorMode::ALL.iter().map(|m| m.name()).collect();
    eprintln!(
        "usage: vital [scan | info | {}] [--csv]\n\
         \n\
           scan        list devices in range and exit\n\
           info        print firmware version, mode and battery, then exit\n\
           eeg         stream all three EEG channels (default)\n\
           impedance   stream electrode impedance in ohms\n\
           ch1..ch3    stream a single EEG channel\n\
           off         connect but leave the front end idle\n\
         \n\
           --csv       emit CSV on stdout instead of aligned columns",
        modes.join(" | ")
    );
    std::process::exit(2)
}

fn parse_args() -> Command {
    let mut mode: Option<SensorMode> = None;
    let mut csv = false;

    for arg in std::env::args().skip(1) {
        match arg.as_str() {
            "--csv" => csv = true,
            "-h" | "--help" => usage(),
            "scan" => return Command::Scan,
            "info" => return Command::Info,
            // Every streaming mode name comes from the library, so the CLI and
            // the crate cannot drift apart.
            other => match other.parse::<SensorMode>() {
                Ok(parsed) => mode = Some(parsed),
                Err(e) => {
                    eprintln!("{e}");
                    usage()
                }
            },
        }
    }

    Command::Stream {
        mode: mode.unwrap_or(SensorMode::EegAllChannels),
        csv,
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::init();

    match parse_args() {
        Command::Scan => scan().await,
        Command::Info => info().await,
        Command::Stream { mode, csv } => stream(mode, csv).await,
    }
}

async fn scan() -> Result<()> {
    let devices = VitalClient::new(VitalClientConfig::default())
        .scan_all()
        .await?;
    if devices.is_empty() {
        println!("no devices found");
        return Ok(());
    }
    for device in &devices {
        println!("{:<24} {}", device.name, device.id);
    }
    Ok(())
}

async fn info() -> Result<()> {
    // No streams: this only reads, so there is nothing to subscribe to and no
    // reason to make the device start sampling.
    let config = VitalClientConfig {
        enable_eeg: false,
        enable_accelerometer: false,
        ..Default::default()
    };
    let (mut events, handle) = VitalClient::new(config).connect().await?;

    let version = handle.firmware_version().await?;
    println!("firmware   {} (built {})", version.version, version.built);
    match handle.read_mode().await? {
        Some(mode) => println!("mode       {mode}"),
        None => println!("mode       not set since boot"),
    }

    // Battery arrives by notification, so wait for one rather than guess — but
    // bounded, so `info` cannot hang on a device that never sends one.
    let wait = async {
        while let Some(event) = events.recv().await {
            match event {
                VitalEvent::Battery(b) => {
                    println!(
                        "battery    {}%  charger {}  state {}",
                        b.percent(),
                        if b.charger_connected() { "in" } else { "out" },
                        b.charge_state_code()
                    );
                    return;
                }
                VitalEvent::Disconnected => return,
                _ => {}
            }
        }
    };
    if tokio::time::timeout(Duration::from_secs(BATTERY_WAIT_SECS), wait)
        .await
        .is_err()
    {
        println!("battery    no notification within {BATTERY_WAIT_SECS} s");
    }
    handle.disconnect().await
}

async fn stream(mode: SensorMode, csv: bool) -> Result<()> {
    let client = VitalClient::new(VitalClientConfig {
        mode,
        ..Default::default()
    });
    let (mut events, handle) = client.connect().await?;

    match handle.firmware_version().await {
        Ok(v) => eprintln!("firmware {} (built {})", v.version, v.built),
        Err(e) => eprintln!("could not read firmware version: {e}"),
    }

    handle.start().await?;
    eprintln!("streaming in {mode} mode — Ctrl-C to stop");

    if csv {
        print_csv_header(mode);
    }
    let mut estimator = ImpedanceEstimator::with_averaging(IMPEDANCE_AVERAGING_CYCLES);

    while let Some(event) = events.recv().await {
        match event {
            VitalEvent::Eeg(frame) => {
                if csv {
                    // One row per sample, not per frame: a CSV of EEG wants a
                    // uniform time base, and the two samples in a frame are 4 ms
                    // apart.
                    for sample in 0..EEG_SAMPLES_PER_PACKET {
                        print!(
                            "{:.3}",
                            frame.timestamp + sample as f64 * EEG_SAMPLE_INTERVAL_MS
                        );
                        for channel in 0..EEG_CHANNELS {
                            match frame.channel(channel) {
                                Some(s) => print!(",{:.4}", s.uv[sample]),
                                None => print!(","),
                            }
                        }
                        println!();
                    }
                } else {
                    let cells: Vec<String> = frame
                        .enabled()
                        .map(|(channel, s)| {
                            format!(
                                "{}={:>9.2},{:>9.2}",
                                eeg_channel_name(channel),
                                s.uv[0],
                                s.uv[1]
                            )
                        })
                        .collect();
                    println!("eeg  t={:.0}  {}  µV", frame.timestamp, cells.join("  "));
                }
            }
            VitalEvent::Impedance(frame) => {
                let Some(estimate) = estimator.push(&frame) else {
                    continue;
                };
                if csv {
                    print!("{:.3}", estimate.timestamp);
                    for ohms in estimate.ohms() {
                        match ohms {
                            Some(z) => print!(",{z:.0}"),
                            None => print!(","),
                        }
                    }
                    println!();
                } else {
                    let cells: Vec<String> = estimate
                        .ohms()
                        .iter()
                        .enumerate()
                        .filter_map(|(channel, z)| {
                            z.map(|z| {
                                let contact = if estimate.lead_off.positive_off(channel) {
                                    "OFF"
                                } else {
                                    "on"
                                };
                                format!("{}={:>8.0}Ω [{contact}]", eeg_channel_name(channel), z)
                            })
                        })
                        .collect();
                    println!("imp  t={:.0}  {}", estimate.timestamp, cells.join("  "));
                }
            }
            VitalEvent::Accelerometer(a) => {
                if !csv {
                    let [x, y, z] = a.g();
                    println!("acc  {x:+.3} {y:+.3} {z:+.3} g");
                }
            }
            VitalEvent::Battery(b) => {
                if !csv {
                    println!(
                        "bat  {}%  {} mV  charger={}",
                        b.percent_clamped(),
                        b.millivolts(),
                        if b.charger_connected() { "yes" } else { "no" }
                    );
                }
            }
            VitalEvent::FirmwareProgress(p) => eprintln!("ota  {} chunk(s)", p.chunks_accepted),
            VitalEvent::ModeChanged(m) => {
                // Samples from before a mode change must not be averaged with
                // samples from after it.
                estimator.reset();
                eprintln!("mode {m}");
            }
            VitalEvent::Connected(name) => eprintln!("connected to {name}"),
            VitalEvent::Disconnected => {
                eprintln!("disconnected");
                break;
            }
        }
    }
    Ok(())
}

fn print_csv_header(mode: SensorMode) {
    let unit = if mode.is_impedance() { "ohms" } else { "uv" };
    print!("timestamp_ms");
    for channel in 0..EEG_CHANNELS {
        print!(",{}_{unit}", eeg_channel_name(channel));
    }
    println!();
}
