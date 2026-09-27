//! A bounded hand-off from an external IMU worker to the control loop.
//!
//! The worker owns the bus and publishes only the newest complete sample. Readers never
//! perform I2C work and never wait for a producer that is retrying a disconnected sensor.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::imu::ImuData;

#[derive(Debug, Clone, Copy)]
pub struct ExternalImuSample {
    pub data: ImuData,
    pub ready: bool,
    pub sequence: u64,
    pub at: Instant,
}

#[derive(Clone, Debug)]
pub struct ExternalImu {
    inner: Arc<Mutex<Option<ExternalImuSample>>>,
}

impl ExternalImu {
    pub fn new() -> Self {
        Self {
            inner: Arc::new(Mutex::new(None)),
        }
    }

    pub fn publish(&self, data: ImuData, ready: bool, sequence: u64) {
        *self.inner.lock().expect("external IMU mutex poisoned") = Some(ExternalImuSample {
            data,
            ready,
            sequence,
            at: Instant::now(),
        });
    }

    pub fn snapshot(&self, max_age: Duration) -> Option<ExternalImuSample> {
        let sample = *self.inner.lock().expect("external IMU mutex poisoned");
        sample.filter(|sample| sample.at.elapsed() <= max_age)
    }
}

impl Default for ExternalImu {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn latest_sample_replaces_previous_sample() {
        let imu = ExternalImu::new();
        imu.publish(ImuData::default(), true, 1);
        imu.publish(
            ImuData {
                gyro: [1.0, 2.0, 3.0],
                ..ImuData::default()
            },
            false,
            2,
        );

        let sample = imu.snapshot(Duration::from_secs(1)).expect("sample");
        assert_eq!(sample.sequence, 2);
        assert_eq!(sample.data.gyro, [1.0, 2.0, 3.0]);
        assert!(!sample.ready);
    }
}
