# vital

Async Rust client for **Vital Neuro** EEG headphones over Bluetooth Low Energy.

Built on [`webbluetooth`](https://crates.io/crates/webbluetooth), so the same code
runs natively on macOS, iOS, Linux, Windows and Android — and in a browser, via
`wasm32-unknown-unknown`.

Shaped after [`muse-rs`](https://crates.io/crates/muse-rs): scan, connect, read an
`mpsc::Receiver<VitalEvent>`, drive the device through a handle.

## Contents

- [Install](#install)
- [Quick start](#quick-start)
- [CLI](#cli)
- [Three things to know first](#three-things-to-know-first)
- [API](#api)
- [Signals](#signals)
- [The device](#the-device)
- [GATT profile](#gatt-profile)
- [Firmware updates](#firmware-updates)
- [Limits](#limits)
- [Development](#development)
- [Licence](#licence)

## Install

```toml
[dependencies]
vital = "0.0.2"
tokio = { version = "1", features = ["rt-multi-thread", "macros"] }
```

Requires Rust **1.94**, inherited from `webbluetooth`.

> **Keep `webbluetooth` versions in lockstep.** Cargo treats `0.0.x` releases as
> mutually incompatible, so depending on a different `0.0.x` than this crate pulls
> in *two* copies — and `Bluetooth::shared()` memoises into a `OnceLock`, so two
> copies means two BLE sessions and devices that will not connect. The same
> applies to `vital` itself at `0.0.x`.

For a browser build, nothing extra is needed:

```sh
cargo build --lib --target wasm32-unknown-unknown
```

The native-only pieces (the `vital` binary, `env_logger`, tokio's runtime and
timers) are gated by target, so a wasm build picks up the right dependency set
with no feature flags to remember.

## Quick start

This is [`examples/quickstart.rs`](examples/quickstart.rs) verbatim, so it is
compiled on every build and cannot drift from the API.

```rust
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
```

## CLI

```
vital scan                 list every device in range
vital info                 firmware version, mode and battery, then exit
vital eeg                  stream EEG (the default)
vital eeg --csv            the same, as CSV on stdout
vital impedance            electrode impedance in ohms
vital ch1 | ch2 | ch3      one channel only
vital off                  connect but leave the front end idle
```

Mode names come from `SensorMode::ALL`, so `--help` and the parser cannot drift
apart from the library.

`RUST_LOG=info` adds protocol logging on **stderr**, leaving stdout clean:

```sh
RUST_LOG=info vital eeg --csv > session.csv
```

CSV is one row per *sample*, not per notification — a uniform 4 ms time base is
what a plot or a dataframe wants:

```
timestamp_ms,ch1_uv,ch2_uv,ch3_uv
1761068412345.000,-12.3456,4.8291,-0.5364
```

Channels the active mode does not stream are left empty. In `impedance` mode the
columns become `ch1_ohms`, `ch2_ohms`, `ch3_ohms`, one row per estimate.

```sh
cargo run --example stream    # per-channel mean amplitude, once a second
```

## Three things to know first

None of these are obvious from the wire format, and each one is the difference
between a working stream and silence.

### Subscribing is half of starting

The device arms its sampling from the EEG characteristic's **CCCD**, *and* gates
it on the mode written to `0x2D3A`. Neither alone produces data.

`connect()` subscribes and `start()` writes the mode, in that order — which is the
order that works. If you drive the GATT layer yourself, do the same.

### Impedance shares the EEG characteristic

`SensorMode::Impedance` does not open a second stream. The same 20-byte
notifications simply start carrying lead-off measurements instead of EEG, in
identical framing — nothing in the bytes distinguishes them.

The client tracks the mode and emits `VitalEvent::Impedance` rather than
`VitalEvent::Eeg`, so matching on the event type does the right thing. Watch
`VitalEvent::ModeChanged` if you need to know when the meaning switched.

### One event is one frame

A notification carries all three channels, sampled simultaneously. So one event is
one **frame**, not one event per channel:

```rust
VitalEvent::Eeg(frame) => for (channel, samples) in frame.enabled() { /* … */ }
```

This differs from `muse-rs`, where each electrode is its own characteristic and
the split is forced. Here it is not, and splitting would make a consumer count
events back together — which it cannot do correctly without also tracking the
mode.

## API

### Client

| | |
|---|---|
| `VitalClient::scan_all` | full scan window, returns every device found |
| `VitalClient::connect` | first match wins; `scan_timeout_secs` is a deadline |
| `VitalClient::connect_to` | connect to a `VitalDevice` from `scan_all` |

`VitalClientConfig` sets the startup `mode`, which streams to subscribe
(`enable_eeg`, `enable_accelerometer`, `enable_battery`,
`enable_firmware_progress`), `scan_timeout_secs`, and an optional `name_prefix`.
Every stream flag is only the *initial* state.

### Handle

| | |
|---|---|
| mode | `start`, `stop`, `set_mode`, `mode` (local), `read_mode` (from the device) |
| streams, live | `set_eeg_enabled`, `set_accelerometer_enabled`, `set_battery_enabled`, `set_firmware_progress_enabled` |
| device | `firmware_version`, `rename` |
| update | `request_firmware_progress`, `read_firmware_chunks_accepted` |
| link | `is_connected`, `disconnect` |

The accelerometer and battery have no mode command — the subscription is the only
control the device offers, so `set_*_enabled` is how you turn them off
mid-session.

### Events

`VitalEvent` is `Connected`, `Eeg`, `Impedance`, `Accelerometer`, `Battery`,
`FirmwareProgress`, `ModeChanged`, `Disconnected`.

### Modes

`SensorMode` implements `Display` and `FromStr` (with `all` and `imp` aliases,
case-insensitive) and carries `ALL`, `name()`, `channels()`, `channel_count()`,
`channel_mask()`, `encode()`, `from_command()`.

## Signals

### EEG

Three channels at **250 SPS**, two samples per notification, scaled to microvolts
by `EEG_SCALE_UV` (0.536 µV/LSB — the front end runs at **gain 1**, which is
unusually low for scalp EEG, so expect small magnitudes and do your own
filtering).

`EegFrame` carries `timestamp`, `lead_off` and
`channels: [Option<ChannelSamples>; 3]`, with `channel()`, `enabled()` and
`channel_count()`. Each `ChannelSamples` has both `uv: [f64; 2]` and
`raw: [i32; 2]`.

Timestamps are extrapolated host-side from the packet count, anchored at the first
packet and re-anchored if they drift more than a second from wall time. These
packets carry **no device-side sequence counter**, so a dropped notification is
undetectable — treat `timestamp` as a smoothed arrival time, not a device clock.

### Impedance

Impedance samples alternate at 62.5 Hz: a single sample is a point on a sine wave
and means little by itself. `ImpedanceEstimator` demodulates them into one number
per electrode:

```rust
let mut estimator = ImpedanceEstimator::with_averaging(16);   // ~256 ms
if let Some(estimate) = estimator.push(&frame) {
    println!("{:?} Ω   worst: {:?}", estimate.ohms(), estimate.worst());
}
```

Excitation is at exactly `f_DR/4` while sampling is at `f_DR`, so there are
exactly **4 samples per cycle, 90° apart**. A 4-point quadrature demodulation is
then exact — no window, no leakage — and it cancels the electrode's DC offset,
which is far larger than the signal, rather than filtering it.

`amplitude_codes` is the unscaled measurement; `ohms()` applies
`IMPEDANCE_OHMS_PER_CODE`, which is *derived* rather than measured — see
[Limits](#limits).

### Lead-off status

The 2-byte packet header is the front end's two lead-off bytes, un-straddled from
the nibble boundary they sit on inside its 24-bit status word. Bit *n* is channel
*n*. Read it from the `lead_off` field of `EegFrame` / `ImpedanceFrame`.

It is populated **only in impedance mode**, and only `positive_off()` carries
information:

- In EEG mode `LOFF_SENSP` and `LOFF_SENSN` are both zero, switching detection off
  entirely.
- In impedance mode `LOFF_SENSP` is `0x07` while `LOFF_SENSN` stays `0x00`, so the
  negative-input detector never runs.

This is the calibration-free answer to "is this electrode on?", and a useful
cross-check on the computed magnitude.

### Accelerometer

A single 3-axis sample per notification, big-endian `i16`, data left-justified.
`counts()` gives the 12-bit values; `g()` applies `ACCELEROMETER_SCALE_G`.

> `g()` reproduces the **device's own** conversion — 1 mg per 12-bit count — but
> the range register is set to ±4 g in normal (10-bit) mode, which is 2 mg per
> count by the part's datasheet. The two disagree by a factor of two. The crate
> follows the device so that `g()` matches the figure the device itself reports;
> double it if you want datasheet-correct g.

### Battery

`millivolts()`, `percent()`, `percent_clamped()`, `percent_from_millivolts()`,
`charger_connected()`, `charge_status_pins()`, `charge_state_code()`,
`charge_finished()`.

> **`percent()` can exceed 100.** The device's curve is
> `(millivolts − 3150) / 10`, which reaches 100 at 4150 mV, but its clamp only
> engages at 4200 mV — so 4150–4199 mV reports 101 to 104. Use
> `percent_clamped()` unless you specifically want the device's own number.

## The device

| | |
|---|---|
| Link | dual-mode — BLE for EEG, Bluetooth Classic A2DP for audio |
| Front end | TI ADS1299, 3 channels at 250 SPS, gain 1 |
| Also on board | LIS3DH accelerometer, battery/charger telemetry |

This crate speaks only the BLE side. Audio is a separate link — pair the
headphones with the OS as normal and the two coexist.

## GATT profile

One service, `0x2D0D`, named in the advertisement (`03 03 0d 2d`). That is what
this crate filters on: the device name is stored on the unit and is
user-settable, so no name prefix reliably matches. All UUIDs are 16-bit assigned
numbers expanded onto the SIG base.

| UUID | Usable as | Meaning |
|---|---|---|
| `0x2D37` | notify | EEG samples — 20 bytes |
| `0x2D3A` | read + write | sensor mode, 2 bytes **big-endian** |
| `0x2D3B` | write | rename (device reboots) |
| `0x2D47` | notify | accelerometer — 6 bytes |
| `0x2D57` | notify | battery — 7 bytes |
| `0x2D58` | read | firmware version ‖ build date |
| `0x2D60` | read + write + notify | firmware-update control, progress count |
| `0x2D61` | write, write-NR | firmware-update payload |

Two quirks worth knowing:

- The three streaming characteristics *declare* the Read property, but their value
  attributes carry no read permission — reads fail. Subscribe; do not poll.
- The mode and control characteristics really are readable, and hold a 4-byte
  placeholder (`11 22 33 44`) until first written. `read_mode()` and
  `read_firmware_chunks_accepted()` return `None` for it rather than decoding
  nonsense.

### Sensor modes

Written to `0x2D3A` as 2 bytes **big-endian** — note that, since the CCCDs around
it are little-endian as usual.

| Command | `SensorMode` | Channels |
|---|---|---|
| `0x0000`, `0x0001` | `EegAllChannels` | 1, 2, 3 |
| `0x0002` | `EegChannel1` | 1 |
| `0x0003` | `EegChannel2` | 2 |
| `0x0004` | `EegChannel3` | 3 |
| `0x0005` | `Impedance` | 1, 2, 3 — AC lead-off, 12 nA at `f_DR/4`, gain 24 |
| `≥ 0x0006` | `Off` | none |

### EEG and impedance packet

```
byte  0..2    [LOFF_STATP, LOFF_STATN] — per-channel electrode-off bits
byte  2..11   sample 0:  ch1[3] ch2[3] ch3[3]   24-bit big-endian, signed
byte 11..20   sample 1:  ch1[3] ch2[3] ch3[3]
```

Channels outside the active mask are filled with `0xFF`. `EegPacket::present`
reports them absent, and a frame's `channels` entry is `None`. Both samples must
be all-ones for a channel to count as absent, so a genuinely saturated `-1`
reading is not mistaken for filler.

### Battery packet

```
byte 0  CHG_STAT1 pin (GPIO 39)      byte 4  battery millivolts, low byte
byte 1  CHG_STAT2 pin (GPIO 34)      byte 5  state of charge, percent
byte 2  CHG_nPG   pin (GPIO 4, low)  byte 6  charge-state flag
byte 3  battery millivolts, high byte
```

Byte 6 is a boolean despite looking like a code: `0` when
`nPG == 0 && STAT1 == 1 && STAT2 == 1`, else `1`.

## Firmware updates

`protocol::firmware` has the complete wire format — `OTA_BEGIN` / `OTA_END`,
`encode_chunk` (4-byte little-endian index starting at **1**, then payload), and
`decode_progress`.

There is deliberately **no one-call flasher**. An interrupted or mis-framed update
leaves the device booting from a half-written partition, and that should not be
reachable by accident. The pieces are complete enough to drive an update
deliberately.

## Limits

Every notification payload is decoded. One thing is not *calibrated*:
`IMPEDANCE_OHMS_PER_CODE` is computed from the ADS1299 datasheet and the device's
configured gain and excitation current, not measured against a reference.

- Relative comparisons between electrodes are unaffected.
- `ImpedanceEstimate::amplitude_codes` is the unscaled figure, if you would rather
  calibrate against a known resistor.
- `lead_off()` answers "is this electrode on?" with no calibration at all.

The accelerometer scale carries the factor-of-two caveat described
[above](#accelerometer). Nothing else is inferred.

## Development

```sh
cargo test                                          # 53 tests
cargo clippy --all-targets -- -D warnings
cargo fmt --check
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps       # intra-doc links are checked
cargo check --lib --target wasm32-unknown-unknown    # the browser target really builds
```

53 tests: 30 unit, 16 integration (`tests/decode.rs`, the full decode path from
notification bytes with no BLE involved), 7 doctests. The crate denies
`missing_docs`, `missing_debug_implementations` and broken intra-doc links.

## Licence

Apache-2.0 — see [LICENSE](LICENSE).
