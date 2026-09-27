//! Linux worker for a BNO085/BNO088 connected through I2C.
//!
//! This is intentionally a small adapter around the open-source `bno080` SHTP driver. The
//! Linux device is opened and polled here, never from the control loop. The current upstream
//! I2C API exposes the fused rotation vector; angular velocity is reconstructed from successive
//! quaternions until a native gyro-report API is selected.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use bno080::interface::I2cInterface;
use bno080::wrapper::BNO080;
use i2cdev::core::I2CDevice;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};

use crate::external_imu::ExternalImu;
use crate::imu::ImuData;

const WARMUP_SAMPLES: u64 = 25;

struct LinuxI2c {
    device: LinuxI2CDevice,
}

impl LinuxI2c {
    fn open(path: &Path, address: u8) -> Result<Self, LinuxI2CError> {
        Ok(Self {
            device: LinuxI2CDevice::new(path, address as u16)?,
        })
    }
}

impl embedded_hal_02::blocking::i2c::Write for LinuxI2c {
    type Error = LinuxI2CError;

    fn write(&mut self, _address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.device.write(bytes)
    }
}

impl embedded_hal_02::blocking::i2c::Read for LinuxI2c {
    type Error = LinuxI2CError;

    fn read(&mut self, _address: u8, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.device.read(bytes)
    }
}

impl embedded_hal_02::blocking::i2c::WriteRead for LinuxI2c {
    type Error = LinuxI2CError;

    fn write_read(
        &mut self,
        _address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.device.write(write)?;
        self.device.read(read)
    }
}

struct Delay;

impl embedded_hal_02::blocking::delay::DelayMs<u8> for Delay {
    fn delay_ms(&mut self, ms: u8) {
        thread::sleep(Duration::from_millis(ms as u64));
    }
}

pub struct Bno08xWorker {
    stop: Arc<AtomicBool>,
    join: Option<JoinHandle<()>>,
}

impl Bno08xWorker {
    pub fn spawn(path: impl Into<PathBuf>, address: u8, hz: u32, output: ExternalImu) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let thread_stop = stop.clone();
        let path = path.into();
        let hz = hz.max(1).min(1000);
        let join = thread::Builder::new()
            .name("walking-imu".to_owned())
            .spawn(move || run(path, address, hz, output, thread_stop))
            .expect("spawn walking IMU worker");
        Self {
            stop,
            join: Some(join),
        }
    }
}

impl Drop for Bno08xWorker {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

fn run(path: PathBuf, address: u8, hz: u32, output: ExternalImu, stop: Arc<AtomicBool>) {
    let period = Duration::from_secs_f64(1.0 / hz as f64);
    let mut sequence = 0;
    let mut previous: Option<([f64; 4], Instant)> = None;

    while !stop.load(Ordering::Relaxed) {
        let result = match LinuxI2c::open(&path, address) {
            Ok(i2c) => run_device(
                i2c,
                address,
                period,
                &output,
                &stop,
                &mut sequence,
                &mut previous,
            ),
            Err(error) => Err(error.to_string()),
        };
        if let Err(error) = result {
            tracing::warn!(error = %error, path = %path.display(), address, "walking IMU unavailable; retrying");
        }
        if !stop.load(Ordering::Relaxed) {
            thread::sleep(Duration::from_millis(500));
        }
    }
}

fn run_device(
    i2c: LinuxI2c,
    address: u8,
    period: Duration,
    output: &ExternalImu,
    stop: &AtomicBool,
    sequence: &mut u64,
    previous: &mut Option<([f64; 4], Instant)>,
) -> Result<(), String> {
    let mut imu = BNO080::new_with_interface(I2cInterface::new(i2c, address));
    let mut delay = Delay;
    imu.init(&mut delay).map_err(|e| format!("init: {e:?}"))?;
    imu.enable_rotation_vector(period.as_millis().clamp(1, u16::MAX as u128) as u16)
        .map_err(|e| format!("enable rotation vector: {e:?}"))?;

    let mut ready_samples = 0u64;
    while !stop.load(Ordering::Relaxed) {
        let handled = imu.handle_all_messages(&mut delay, 5);
        if handled == 0 {
            thread::sleep(period.min(Duration::from_millis(5)));
            continue;
        }
        let q = imu
            .rotation_quaternion()
            .map_err(|e| format!("rotation vector: {e:?}"))?;
        let q = [q[3] as f64, q[0] as f64, q[1] as f64, q[2] as f64];
        let now = Instant::now();
        let gyro = previous
            .replace((q, now))
            .and_then(|(old, at)| quaternion_rate(old, q, now.duration_since(at)))
            .unwrap_or([0.0; 3]);
        let gravity = rotate_inverse(q, [0.0, 0.0, -1.0]);
        ready_samples = ready_samples.saturating_add(1);
        *sequence = sequence.saturating_add(1);
        output.publish(
            ImuData {
                gyro,
                gravity,
                quat: q,
            },
            ready_samples >= WARMUP_SAMPLES,
            *sequence,
        );
    }
    Ok(())
}

fn quaternion_rate(old: [f64; 4], new: [f64; 4], dt: Duration) -> Option<[f64; 3]> {
    let seconds = dt.as_secs_f64();
    if seconds <= 0.0 || seconds > 0.2 {
        return None;
    }
    let dot = old[0] * new[0] + old[1] * new[1] + old[2] * new[2] + old[3] * new[3];
    let sign = if dot < 0.0 { -1.0 } else { 1.0 };
    Some([
        2.0 * sign * (new[1] - old[1]) / seconds,
        2.0 * sign * (new[2] - old[2]) / seconds,
        2.0 * sign * (new[3] - old[3]) / seconds,
    ])
}

fn rotate_inverse(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [w, x, y, z] = q;
    let t = [
        2.0 * (y * v[2] - z * v[1]),
        2.0 * (z * v[0] - x * v[2]),
        2.0 * (x * v[1] - y * v[0]),
    ];
    [
        v[0] - w * t[0] + y * t[2] - z * t[1],
        v[1] - w * t[1] + z * t[0] - x * t[2],
        v[2] - w * t[2] + x * t[1] - y * t[0],
    ]
}
