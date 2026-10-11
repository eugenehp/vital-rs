//! Scanning, connecting, and the command handle.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, Context, Result};
use log::{debug, info, warn};
use tokio::sync::mpsc;
use uuid::Uuid;
use webbluetooth::chooser::{Candidate, FirstMatch};
use webbluetooth::stream::StreamExt;
use webbluetooth::uuid::BluetoothUuid;
use webbluetooth::{
    Bluetooth, BluetoothDevice, DeviceFilter, Grant, LeScanOptions, RemoteGattCharacteristic,
    RemoteGattServer, RequestDeviceOptions,
};

use crate::clock::PacketClock;
use crate::parse::{decode_accelerometer, decode_battery, decode_eeg_packet};
use crate::platform;
use crate::protocol::{
    firmware, parse_firmware_version, FirmwareVersion, SensorMode, ACCELEROMETER_CHARACTERISTIC,
    BATTERY_CHARACTERISTIC, EEG_CHARACTERISTIC, FIRMWARE_CONTROL_CHARACTERISTIC,
    FIRMWARE_VERSION_CHARACTERISTIC, MAX_DEVICE_NAME_LEN, RENAME_CHARACTERISTIC,
    SENSOR_MODE_CHARACTERISTIC, UNWRITTEN_ATTRIBUTE_VALUE, VITAL_SERVICE_UUID,
};
use crate::types::{
    AccelerometerReading, BatteryReading, EegFrame, FirmwareProgress, ImpedanceFrame, VitalEvent,
};

// ── GATT helpers ──────────────────────────────────────────────────────────────

// The adapter session is `Bluetooth::shared()`: one per process, cloned by every
// caller. It has to be shared, because a `VitalDevice` is only usable through
// the session that found it, and a caller may well scan with one `VitalClient`
// and connect with another.

/// What a device request asks for.
///
/// Filtering on the **service** rather than the name, because the device's name
/// lives in its NVS and is user-settable — there is no prefix that reliably
/// matches one — while `0x2D0D` is in the advertisement (`03 03 0d 2d`) on every
/// unit. A name prefix is applied on top only if the caller asked for one.
fn request_options(name_prefix: Option<&str>) -> Result<RequestDeviceOptions> {
    let mut filter = DeviceFilter::new().service(VITAL_SERVICE_UUID)?;
    if let Some(prefix) = name_prefix {
        filter = filter.name_prefix(prefix);
    }
    Ok(RequestDeviceOptions::new()
        .filter(filter)
        .optional_service(VITAL_SERVICE_UUID)?)
}

/// What a scan reports. A scan grants nothing, so this is only about matching.
fn scan_options(name_prefix: Option<&str>) -> Result<LeScanOptions> {
    let mut filter = DeviceFilter::new().service(VITAL_SERVICE_UUID)?;
    if let Some(prefix) = name_prefix {
        filter = filter.name_prefix(prefix);
    }
    Ok(LeScanOptions::new().filter(filter))
}

/// What adopting a scanned device grants: the one service, no manufacturer data.
fn vital_grant() -> Result<Grant> {
    Ok(Grant::new().service(VITAL_SERVICE_UUID)?)
}

/// Pick one characteristic out of a service's discovered set.
///
/// The device's attribute table declares some UUIDs twice — a second
/// read-only attribute holding a label string like `"EEG Samples"` shares the
/// characteristic's own UUID, because the author used that in place of
/// `0x2901`. So a UUID lookup can match more than one attribute. Taking the
/// first match is correct: the characteristic declaration precedes its label in
/// the table, so discovery reports the real one first.
fn find_char(chars: &[RemoteGattCharacteristic], uuid: Uuid) -> Result<RemoteGattCharacteristic> {
    let want = BluetoothUuid::from(uuid);
    chars
        .iter()
        .find(|c| *c.uuid() == want)
        .cloned()
        .ok_or_else(|| anyhow!("characteristic {uuid} not found on this device"))
}

fn now_ms() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system clock is before Unix epoch")
        .as_secs_f64()
        * 1000.0
}

// ── Stream plumbing ───────────────────────────────────────────────────────────

/// What every notification forwarder needs: somewhere to send events, and the
/// means to tell an unsubscribe apart from a dropped link.
#[derive(Clone)]
struct StreamContext {
    events: mpsc::Sender<VitalEvent>,
    gatt: RemoteGattServer,
    /// So that the disconnect watcher and four forwarders, all of which notice a
    /// dropped link, between them emit exactly one
    /// [`VitalEvent::Disconnected`].
    disconnect_reported: Arc<AtomicBool>,
}

impl StreamContext {
    /// Emit [`VitalEvent::Disconnected`], unless someone already has.
    async fn report_disconnect(&self) {
        if self.disconnect_reported.swap(true, Ordering::SeqCst) {
            return;
        }
        let _ = self.events.send(VitalEvent::Disconnected).await;
    }

    /// Called when a notification stream ends.
    ///
    /// A stream ends for two very different reasons — the caller unsubscribed,
    /// or the link dropped — and only the second is a disconnect. Asking the
    /// adapter tells them apart, which is what lets
    /// [`VitalHandle::set_accelerometer_enabled`] turn a sensor off without the
    /// consumer seeing a spurious `Disconnected`.
    async fn stream_ended(&self, what: &str) {
        if self.gatt.connected() {
            debug!("{what} notifications ended (unsubscribed)");
        } else {
            info!("{what} notifications ended — link is down");
            self.report_disconnect().await;
        }
    }
}

/// Subscribe to EEG and forward decoded packets.
///
/// The subscription is not merely how notifications are delivered: the
/// device's CCCD handler is what arms its sampling task, so this has to be in
/// place before a [`SensorMode`] is written or the device stays silent.
async fn subscribe_eeg(
    characteristic: &RemoteGattCharacteristic,
    ctx: StreamContext,
    mode: Arc<AtomicU16>,
) -> Result<()> {
    let mut notifications = characteristic.start_notifications().await?;
    platform::spawn(async move {
        let mut clock = PacketClock::for_eeg();

        while let Some(data) = notifications.next().await {
            let Some(packet) = decode_eeg_packet(&data) else {
                warn!(
                    "EEG notification was {} bytes, expected 20 — dropped",
                    data.len()
                );
                continue;
            };
            // One timestamp for the whole frame: the ADS1299 samples all three
            // channels simultaneously.
            let timestamp = clock.next(now_ms());
            // Impedance arrives on this same characteristic in this same framing;
            // the mode is the only thing that distinguishes them.
            let event = if SensorMode::from_command(mode.load(Ordering::Relaxed)).is_impedance() {
                VitalEvent::Impedance(ImpedanceFrame::from_packet(&packet, timestamp))
            } else {
                VitalEvent::Eeg(EegFrame::from_packet(&packet, timestamp))
            };

            if ctx.events.send(event).await.is_err() {
                return; // receiver dropped; nothing left to feed
            }
        }
        ctx.stream_ended("EEG").await;
    });
    Ok(())
}

async fn subscribe_accelerometer(
    characteristic: &RemoteGattCharacteristic,
    ctx: StreamContext,
) -> Result<()> {
    let mut notifications = characteristic.start_notifications().await?;
    platform::spawn(async move {
        while let Some(data) = notifications.next().await {
            match decode_accelerometer(&data) {
                Some(raw) => {
                    let reading = AccelerometerReading {
                        timestamp: now_ms(),
                        raw,
                    };
                    if ctx
                        .events
                        .send(VitalEvent::Accelerometer(reading))
                        .await
                        .is_err()
                    {
                        return;
                    }
                }
                None => warn!(
                    "accelerometer notification was {} bytes, expected 6 — dropped",
                    data.len()
                ),
            }
        }
        ctx.stream_ended("accelerometer").await;
    });
    Ok(())
}

async fn subscribe_battery(
    characteristic: &RemoteGattCharacteristic,
    ctx: StreamContext,
) -> Result<()> {
    let mut notifications = characteristic.start_notifications().await?;
    platform::spawn(async move {
        while let Some(data) = notifications.next().await {
            match decode_battery(&data) {
                Some(raw) => {
                    let reading = BatteryReading {
                        timestamp: now_ms(),
                        raw,
                    };
                    if ctx.events.send(VitalEvent::Battery(reading)).await.is_err() {
                        return;
                    }
                }
                None => warn!(
                    "battery notification was {} bytes, expected 7 — dropped",
                    data.len()
                ),
            }
        }
        ctx.stream_ended("battery").await;
    });
    Ok(())
}

async fn subscribe_firmware_progress(
    characteristic: &RemoteGattCharacteristic,
    ctx: StreamContext,
) -> Result<()> {
    let mut notifications = characteristic.start_notifications().await?;
    platform::spawn(async move {
        while let Some(data) = notifications.next().await {
            if let Some(chunks_accepted) = firmware::decode_progress(&data) {
                if ctx
                    .events
                    .send(VitalEvent::FirmwareProgress(FirmwareProgress {
                        chunks_accepted,
                    }))
                    .await
                    .is_err()
                {
                    return;
                }
            }
        }
        ctx.stream_ended("firmware control").await;
    });
    Ok(())
}

// ── VitalDevice ───────────────────────────────────────────────────────────────

/// A headphone discovered during a scan.
///
/// Returned by [`VitalClient::scan_all`]; pass to [`VitalClient::connect_to`].
#[derive(Clone, Debug)]
pub struct VitalDevice {
    /// Advertised name. Set on the device itself and stored in its NVS, so it is
    /// whatever someone last renamed it to — not a stable model string.
    pub name: String,
    /// Platform BLE identifier.
    ///
    /// On macOS and iOS this is a per-host UUID the system assigns, so the same
    /// unit has a different id on a different machine; elsewhere it is the
    /// Bluetooth address.
    pub id: String,
    pub(crate) device: BluetoothDevice,
}

// ── VitalClientConfig ─────────────────────────────────────────────────────────

/// Configuration for [`VitalClient`].
///
/// Every stream flag here is only the *initial* state; all of them can be
/// changed on a live connection with the `set_*_enabled` methods on
/// [`VitalHandle`].
#[derive(Debug, Clone)]
pub struct VitalClientConfig {
    /// The mode [`VitalHandle::start`] selects. Default:
    /// [`SensorMode::EegAllChannels`].
    pub mode: SensorMode,
    /// Subscribe to EEG at connect time. Default: `true`.
    ///
    /// Turning this off means [`VitalHandle::start`] will write a mode the
    /// device then has no armed stream for, and no samples arrive. It exists for
    /// callers that only want telemetry.
    pub enable_eeg: bool,
    /// Subscribe to the accelerometer. Default: `true`.
    pub enable_accelerometer: bool,
    /// Subscribe to battery telemetry. Default: `true`.
    pub enable_battery: bool,
    /// Subscribe to firmware-update progress. Default: `false`.
    ///
    /// Only useful while actually driving an update, so off by default.
    pub enable_firmware_progress: bool,
    /// How long [`VitalClient::scan_all`] collects advertisements, and the
    /// deadline [`VitalClient::connect`] gives the first match. Default: `15`.
    pub scan_timeout_secs: u64,
    /// Narrow the match to names starting with this.
    ///
    /// `None` — the default — matches on the advertised service UUID alone,
    /// which is the reliable discriminator here. Set this only to pick one unit
    /// out of several.
    pub name_prefix: Option<String>,
}

impl Default for VitalClientConfig {
    fn default() -> Self {
        Self {
            mode: SensorMode::EegAllChannels,
            enable_eeg: true,
            enable_accelerometer: true,
            enable_battery: true,
            enable_firmware_progress: false,
            scan_timeout_secs: 15,
            name_prefix: None,
        }
    }
}

// ── VitalClient ───────────────────────────────────────────────────────────────

/// BLE client for Vital Neuro headphones.
///
/// Scans, connects, subscribes, and dispatches decoded notifications onto an
/// `mpsc` channel. The returned [`VitalHandle`] is how you change what the
/// device is doing.
///
/// Note that these headphones are a dual-mode device: the EEG side is BLE, and
/// audio is a separate Bluetooth Classic A2DP link. This crate only speaks to
/// the BLE side. If you also want audio, pair the headphones with the OS
/// normally — the two links coexist.
#[derive(Debug, Clone, Default)]
pub struct VitalClient {
    config: VitalClientConfig,
}

impl VitalClient {
    /// A client that will scan and connect per `config`.
    pub fn new(config: VitalClientConfig) -> Self {
        Self { config }
    }

    /// The configuration this client was built with.
    pub fn config(&self) -> &VitalClientConfig {
        &self.config
    }

    /// Scan for **all** nearby devices and return them.
    ///
    /// Runs for the full `scan_timeout_secs` so that several units in range are
    /// all found before returning. Every device returned has already been
    /// granted the service, so [`VitalClient::connect_to`] does not scan again.
    pub async fn scan_all(&self) -> Result<Vec<VitalDevice>> {
        let bluetooth = Bluetooth::shared();
        bluetooth
            .availability()
            .await
            .context("Bluetooth is not usable")?;

        info!(
            "scan_all: scanning for {} s …",
            self.config.scan_timeout_secs
        );
        let mut scan = bluetooth
            .request_le_scan(scan_options(self.config.name_prefix.as_deref())?)
            .await?;

        // A device advertises repeatedly; keyed by id, each one is kept once.
        let mut seen: BTreeMap<String, Candidate> = BTreeMap::new();
        let window = Duration::from_secs(self.config.scan_timeout_secs);
        let _ = platform::timeout(window, async {
            while let Some(candidate) = scan.next().await {
                if !seen.contains_key(&candidate.id) {
                    info!("scan_all: found {} id={}", candidate.label(), candidate.id);
                }
                seen.insert(candidate.id.clone(), candidate);
            }
        })
        .await;
        scan.stop(); // stop the radio before connecting to anything

        // A scan grants nothing, so each sighting is adopted into a device with
        // the grant it will need once connected.
        let grant = vital_grant()?;
        let mut found = vec![];
        for (id, candidate) in seen {
            let name = candidate.label().to_owned();
            match bluetooth.adopt_candidate(&candidate, grant.clone()).await {
                Ok(device) => found.push(VitalDevice { name, id, device }),
                Err(e) => warn!("scan_all: {name} ({id}) could not be adopted: {e}"),
            }
        }
        info!("scan_all: {} device(s) found", found.len());
        Ok(found)
    }

    /// Connect to a device from [`VitalClient::scan_all`] and start dispatching.
    pub async fn connect_to(
        &self,
        device: VitalDevice,
    ) -> Result<(mpsc::Receiver<VitalEvent>, VitalHandle)> {
        self.setup_device(device.device, device.name).await
    }

    /// Scan for the first matching device, connect, and start dispatching.
    ///
    /// Returns as soon as something matching advertises, so `scan_timeout_secs`
    /// acts as a deadline rather than a fixed window. Use this when one unit is
    /// expected; use [`VitalClient::scan_all`] when you need to choose.
    pub async fn connect(&self) -> Result<(mpsc::Receiver<VitalEvent>, VitalHandle)> {
        let bluetooth = Bluetooth::shared();
        bluetooth
            .availability()
            .await
            .context("Bluetooth is not usable")?;

        info!(
            "scanning for a Vital device (timeout: {} s) …",
            self.config.scan_timeout_secs
        );
        let chooser = FirstMatch::with_timeout(Duration::from_secs(self.config.scan_timeout_secs));
        let device = bluetooth
            .request_device_with(
                request_options(self.config.name_prefix.as_deref())?,
                &chooser,
            )
            .await
            .map_err(|e| {
                anyhow!(
                    "no Vital device found within {} s: {e}",
                    self.config.scan_timeout_secs
                )
            })?;

        let name = device.name().unwrap_or_else(|| device.id().to_owned());
        info!("found device: {name}");
        self.setup_device(device, name).await
    }

    /// Connect, subscribe, and build the handle.
    async fn setup_device(
        &self,
        device: BluetoothDevice,
        device_name: String,
    ) -> Result<(mpsc::Receiver<VitalEvent>, VitalHandle)> {
        let gatt = device.gatt();

        // `connect()` has no deadline of its own — it waits for the peripheral
        // for as long as that takes — so one is imposed here.
        platform::timeout(Duration::from_secs(10), gatt.connect())
            .await
            .map_err(|_| anyhow!("BLE connect() timed out after 10 s"))??;

        // One service holds everything, so a single discovery pass is the lot.
        let service = platform::timeout(
            Duration::from_secs(15),
            gatt.get_primary_service(VITAL_SERVICE_UUID),
        )
        .await
        .map_err(|_| anyhow!("service discovery timed out after 15 s"))??;
        let chars = service.get_characteristics(None).await?;
        info!(
            "{device_name}: connected, {} characteristic(s) discovered",
            chars.len()
        );

        // Required: without these there is nothing to drive.
        let eeg_char = find_char(&chars, EEG_CHARACTERISTIC)?;
        let mode_char = find_char(&chars, SENSOR_MODE_CHARACTERISTIC)?;
        let version_char = find_char(&chars, FIRMWARE_VERSION_CHARACTERISTIC)?;
        // Optional: not every device exposes these.
        let accelerometer_char = find_char(&chars, ACCELEROMETER_CHARACTERISTIC).ok();
        let battery_char = find_char(&chars, BATTERY_CHARACTERISTIC).ok();
        let rename_char = find_char(&chars, RENAME_CHARACTERISTIC).ok();
        let firmware_control_char = find_char(&chars, FIRMWARE_CONTROL_CHARACTERISTIC).ok();

        let (tx, rx) = mpsc::channel::<VitalEvent>(256);
        let _ = tx.send(VitalEvent::Connected(device_name.clone())).await;

        // Shared with the EEG forwarder so it can tell EEG bytes from impedance
        // bytes. Starts at `Off`: nothing has been written yet.
        let mode = Arc::new(AtomicU16::new(SensorMode::Off.command()));

        let ctx = StreamContext {
            events: tx.clone(),
            gatt: gatt.clone(),
            disconnect_reported: Arc::new(AtomicBool::new(false)),
        };

        // ── Disconnect watcher ────────────────────────────────────────────────
        // Usually notices before the notification streams do.
        let watcher_ctx = ctx.clone();
        let device_id = device.id().to_owned();
        let mut disconnects = device.watch_disconnect();
        platform::spawn(async move {
            if disconnects.next().await.is_some() {
                info!("device {device_id} disconnected");
                watcher_ctx.report_disconnect().await;
            }
        });

        // ── Initial subscriptions ─────────────────────────────────────────────
        // EEG first, because its CCCD is what arms the sampling task.
        if self.config.enable_eeg {
            subscribe_eeg(&eeg_char, ctx.clone(), Arc::clone(&mode)).await?;
        }
        if self.config.enable_accelerometer {
            if let Some(c) = &accelerometer_char {
                if let Err(e) = subscribe_accelerometer(c, ctx.clone()).await {
                    warn!("accelerometer: {e}");
                }
            }
        }
        if self.config.enable_battery {
            if let Some(c) = &battery_char {
                if let Err(e) = subscribe_battery(c, ctx.clone()).await {
                    warn!("battery: {e}");
                }
            }
        }
        if self.config.enable_firmware_progress {
            if let Some(c) = &firmware_control_char {
                if let Err(e) = subscribe_firmware_progress(c, ctx.clone()).await {
                    warn!("firmware control: {e}");
                }
            }
        }

        let handle = VitalHandle {
            gatt,
            eeg_char,
            mode_char,
            version_char,
            accelerometer_char,
            battery_char,
            rename_char,
            firmware_control_char,
            mode,
            default_mode: self.config.mode,
            ctx,
        };

        Ok((rx, handle))
    }
}

// ── VitalHandle ───────────────────────────────────────────────────────────────

/// Commands for a live connection.
///
/// Returned alongside the event receiver by [`VitalClient::connect`]. Dropping it
/// does not disconnect — the link lives as long as the adapter session holds it, so
/// call [`VitalHandle::disconnect`] when you mean to let go.
pub struct VitalHandle {
    gatt: RemoteGattServer,
    eeg_char: RemoteGattCharacteristic,
    mode_char: RemoteGattCharacteristic,
    version_char: RemoteGattCharacteristic,
    accelerometer_char: Option<RemoteGattCharacteristic>,
    battery_char: Option<RemoteGattCharacteristic>,
    rename_char: Option<RemoteGattCharacteristic>,
    firmware_control_char: Option<RemoteGattCharacteristic>,
    mode: Arc<AtomicU16>,
    default_mode: SensorMode,
    ctx: StreamContext,
}

// `webbluetooth`'s GATT handles are not `Debug`, so this reports the state a
// reader of a log actually wants: what the device is doing and what is wired up.
impl std::fmt::Debug for VitalHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("VitalHandle")
            .field("connected", &self.gatt.connected())
            .field("mode", &self.mode())
            .field("default_mode", &self.default_mode)
            .field("eeg_notifying", &self.eeg_char.is_notifying())
            .field(
                "accelerometer",
                &self.accelerometer_char.as_ref().map(|c| c.is_notifying()),
            )
            .field(
                "battery",
                &self.battery_char.as_ref().map(|c| c.is_notifying()),
            )
            .field("rename_supported", &self.rename_char.is_some())
            .field(
                "firmware_control_supported",
                &self.firmware_control_char.is_some(),
            )
            .finish()
    }
}

impl VitalHandle {
    // ── Sensor mode ───────────────────────────────────────────────────────────

    /// Begin streaming in the mode from [`VitalClientConfig::mode`].
    ///
    /// The subscriptions are already in place by the time `connect` returns, so
    /// this is the single write that starts data flowing.
    pub async fn start(&self) -> Result<()> {
        self.set_mode(self.default_mode).await
    }

    /// Stop the front end — [`SensorMode::Off`].
    ///
    /// The BLE link stays up and the subscriptions stay in place, so
    /// [`VitalHandle::start`] resumes without reconnecting.
    pub async fn stop(&self) -> Result<()> {
        self.set_mode(SensorMode::Off).await
    }

    /// Switch what the front end is doing.
    ///
    /// Also emits [`VitalEvent::ModeChanged`], which is how a consumer learns
    /// whether to expect [`VitalEvent::Eeg`] or [`VitalEvent::Impedance`] next.
    pub async fn set_mode(&self, mode: SensorMode) -> Result<()> {
        // Recorded before the write, so a notification that arrives between the
        // write landing and this store cannot be classified under the old mode.
        self.mode.store(mode.command(), Ordering::Relaxed);
        // With response: the characteristic's only write property is the
        // acknowledged one, and a silent failure here looks exactly like a
        // device that is connected but sending nothing.
        self.mode_char
            .write_value_with_response(&mode.encode())
            .await
            .with_context(|| format!("failed to set sensor mode to {mode:?}"))?;
        info!(
            "sensor mode set to {mode:?} (command 0x{:04x}, channel mask 0b{:03b})",
            mode.command(),
            mode.channel_mask()
        );
        let _ = self.ctx.events.send(VitalEvent::ModeChanged(mode)).await;
        Ok(())
    }

    /// The mode last written *by this handle*.
    ///
    /// Local and free. Use [`VitalHandle::read_mode`] to ask the device.
    pub fn mode(&self) -> SensorMode {
        SensorMode::from_command(self.mode.load(Ordering::Relaxed))
    }

    /// Read the mode back from the device.
    ///
    /// The mode characteristic is readable, and the BLE stack stores whatever
    /// was last written to it — so this reports what the device actually has,
    /// including a mode set by some other client or before this process started.
    ///
    /// Returns `Ok(None)` when nothing has been written since the device booted:
    /// the attribute then still holds the 4-byte placeholder rather
    /// than a 2-byte command.
    pub async fn read_mode(&self) -> Result<Option<SensorMode>> {
        let bytes = self
            .mode_char
            .read_value()
            .await
            .context("failed to read the sensor mode back")?;
        match <[u8; 2]>::try_from(bytes.as_slice()) {
            Ok(two) => Ok(Some(SensorMode::from_command(u16::from_be_bytes(two)))),
            Err(_) => {
                debug!("mode read returned {bytes:02x?} — not a 2-byte command");
                Ok(None)
            }
        }
    }

    // ── Stream control ────────────────────────────────────────────────────────

    /// Subscribe or unsubscribe the EEG stream.
    ///
    /// Unsubscribing stops the device's sampling task, which is a second,
    /// independent off switch to [`VitalHandle::stop`]. Prefer `stop` for
    /// pausing a session; this is for callers that want the subscription itself
    /// gone.
    pub async fn set_eeg_enabled(&self, enabled: bool) -> Result<()> {
        if enabled == self.eeg_char.is_notifying() {
            return Ok(());
        }
        if enabled {
            subscribe_eeg(&self.eeg_char, self.ctx.clone(), Arc::clone(&self.mode)).await
        } else {
            self.eeg_char
                .stop_notifications()
                .await
                .context("failed to unsubscribe EEG")
        }
    }

    /// Subscribe or unsubscribe the accelerometer stream.
    ///
    /// There is no mode command for the accelerometer — the subscription is the
    /// only control the device offers, so this is how you turn it off.
    pub async fn set_accelerometer_enabled(&self, enabled: bool) -> Result<()> {
        let characteristic = self
            .accelerometer_char
            .as_ref()
            .ok_or_else(|| anyhow!("this device does not expose the accelerometer"))?;
        if enabled == characteristic.is_notifying() {
            return Ok(());
        }
        if enabled {
            subscribe_accelerometer(characteristic, self.ctx.clone()).await
        } else {
            characteristic
                .stop_notifications()
                .await
                .context("failed to unsubscribe the accelerometer")
        }
    }

    /// Subscribe or unsubscribe battery telemetry.
    ///
    /// As with the accelerometer, the subscription is the only control.
    pub async fn set_battery_enabled(&self, enabled: bool) -> Result<()> {
        let characteristic = self
            .battery_char
            .as_ref()
            .ok_or_else(|| anyhow!("this device does not expose battery telemetry"))?;
        if enabled == characteristic.is_notifying() {
            return Ok(());
        }
        if enabled {
            subscribe_battery(characteristic, self.ctx.clone()).await
        } else {
            characteristic
                .stop_notifications()
                .await
                .context("failed to unsubscribe battery telemetry")
        }
    }

    /// Subscribe or unsubscribe firmware-update progress notifications.
    pub async fn set_firmware_progress_enabled(&self, enabled: bool) -> Result<()> {
        let characteristic = self
            .firmware_control_char
            .as_ref()
            .ok_or_else(|| anyhow!("this device does not expose the control characteristic"))?;
        if enabled == characteristic.is_notifying() {
            return Ok(());
        }
        if enabled {
            subscribe_firmware_progress(characteristic, self.ctx.clone()).await
        } else {
            characteristic
                .stop_notifications()
                .await
                .context("failed to unsubscribe firmware progress")
        }
    }

    // ── Device info ───────────────────────────────────────────────────────────

    /// Read the device version and build date.
    pub async fn firmware_version(&self) -> Result<FirmwareVersion> {
        let bytes = self
            .version_char
            .read_value()
            .await
            .context("failed to read the device version")?;
        Ok(parse_firmware_version(&bytes))
    }

    /// Rename the device.
    ///
    /// **The device reboots as soon as this write lands**, which drops the BLE
    /// link — expect [`VitalEvent::Disconnected`] rather than an acknowledgement
    /// of success. The name persists in the device's NVS.
    ///
    /// Names longer than [`MAX_DEVICE_NAME_LEN`] are refused here, because the
    /// device will not refuse them itself — see that constant.
    pub async fn rename(&self, name: &str) -> Result<()> {
        if name.is_empty() {
            // The device rejects this too: it checks the first byte is not NUL
            // and logs "Invalid Name value".
            return Err(anyhow!("device name cannot be empty"));
        }
        if name.len() > MAX_DEVICE_NAME_LEN {
            return Err(anyhow!(
                "device name is {} bytes; the device's buffer holds {MAX_DEVICE_NAME_LEN}",
                name.len()
            ));
        }
        let characteristic = self
            .rename_char
            .as_ref()
            .ok_or_else(|| anyhow!("this device does not expose the rename characteristic"))?;
        warn!("renaming to {name:?} — the device will reboot and the link will drop");
        characteristic
            .write_value_with_response(name.as_bytes())
            .await
            .context("failed to write the new device name")?;
        Ok(())
    }

    // ── Firmware update ───────────────────────────────────────────────────────

    /// Ask the device to notify its accepted-chunk count.
    ///
    /// Only observable with the progress stream subscribed — see
    /// [`VitalHandle::set_firmware_progress_enabled`]. For a one-shot answer
    /// that needs no subscription, use
    /// [`VitalHandle::read_firmware_chunks_accepted`].
    pub async fn request_firmware_progress(&self) -> Result<()> {
        let characteristic = self
            .firmware_control_char
            .as_ref()
            .ok_or_else(|| anyhow!("this device does not expose the control characteristic"))?;
        characteristic
            .write_value_with_response(&firmware::encode_control(firmware::OTA_QUERY_PROGRESS))
            .await
            .context("failed to request firmware progress")?;
        Ok(())
    }

    /// Read the accepted-chunk count directly.
    ///
    /// The device stores the count into the control characteristic before
    /// notifying it, so a plain read gets the same number without subscribing.
    ///
    /// Returns `Ok(None)` before any update has begun, when the attribute still
    /// holds the device's placeholder — which as a little-endian `u32` would
    /// otherwise read as a count in the billions.
    pub async fn read_firmware_chunks_accepted(&self) -> Result<Option<u32>> {
        let characteristic = self
            .firmware_control_char
            .as_ref()
            .ok_or_else(|| anyhow!("this device does not expose the control characteristic"))?;
        let bytes = characteristic
            .read_value()
            .await
            .context("failed to read the device chunk count")?;
        if bytes == UNWRITTEN_ATTRIBUTE_VALUE {
            return Ok(None);
        }
        Ok(firmware::decode_progress(&bytes))
    }

    // ── Link ──────────────────────────────────────────────────────────────────

    /// Whether the adapter still considers the peripheral connected.
    ///
    /// Reads cached adapter state, so it neither blocks nor awaits — safe to poll
    /// as a watchdog, and it notices a dropped link sooner than the notification
    /// streams closing.
    pub fn is_connected(&self) -> bool {
        self.gatt.connected()
    }

    /// Stop the front end and drop the link.
    ///
    /// Switching the sensors off first so the device is not left sampling into a
    /// closed connection. A failure to do so is logged, not propagated —
    /// disconnecting is the part the caller asked for.
    pub async fn disconnect(&self) -> Result<()> {
        if let Err(e) = self.set_mode(SensorMode::Off).await {
            warn!("could not switch sensors off before disconnecting: {e}");
        }
        self.gatt.disconnect();
        Ok(())
    }
}
