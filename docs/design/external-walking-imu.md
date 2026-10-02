# External BNO08x Walking IMU
default address: 0x28

Status: implementation started, hardware validation required

## Decision

The walking IMU may be moved off the Dynamixel bus onto the Radxa Zero 3W I2C3 bus.
The supported first implementation is a BNO085 or BNO088 connected over I2C at the
sensor's standard address, with its SHTP transport handled by the open-source Rust
`bno080` crate (BSD-3-Clause). BNO085 and BNO088 use the same BNO08x/SHTP report family;
the adapter must still verify the product ID during bring-up.

The existing `bno08x-rs` crate was evaluated and is not selected for this path: it is
Apache-2.0 and actively maintained, but its public driver is SPI plus GPIO HINTN/RSTN,
not I2C. It therefore does not match the Zero 3W wiring requested here. The smaller
`bno08x-rvc` crate only parses UART RVC frames and is also not a fit.

## Current topology

Today `robotd` owns one `RobotIo`. `DynamixelIo::read()` sends one Protocol 2.0
`sync_read` for ID 200 (`imu_to_dxl`) followed by the 15 servos. `Sensors` then feeds
the policy, fall detector, odometry and telemetry. A missing or malformed ID 200 packet
therefore prevents the whole control loop from obtaining a fresh sample.

The BMI088 mentioned by `deploy/audio/i2c3-pihat.dts` is a different, optional head IMU
owned by `tofd`. It is not the walking IMU and disabling it does not fix an ID 200 bus
failure.

## Target topology

```text
                    +----------------------+
                    | robotd control loop  |
                    | 50 Hz, never blocks |
                    +----------+-----------+
                               |
                latest sample, bounded channel
                               |
                    +----------+-----------+
                    | BNO08x I2C worker    |
                    | init/retry/parse     |
                    +----------+-----------+
                               |
                        /dev/i2c-pihat
                               |
                    BNO085 or BNO088 on I2C3

       /dev/ttyS2: 15 Dynamixel servos only
```

The worker owns the I2C file descriptor and all BNO08x state. The control loop only
copies the newest sample and never performs I2C operations. A bounded latest-value
channel prevents a slow or disconnected sensor from building a queue and affecting the
50 Hz schedule.

## Sensor contract

The adapter publishes the existing `ImuData` contract:

* `quat`: BNO08x game rotation vector, converted to scalar-first `[w, x, y, z]`.
* `gravity`: gravity vector derived from that quaternion in the robot trunk frame.
* `gyro`: angular velocity in rad/s. The selected `bno080` API exposes the rotation
  vector but not a gyro report. Until a lower-level BNO08x I2C driver is selected or
  contributed, the adapter estimates angular velocity from successive quaternions.
  The estimate is explicitly marked as a compatibility limitation and must be validated
  against a hardware log before enabling aggressive fall thresholds.

The mounting transform is configuration, not a hidden constant. The initial default is
the identity transform; the robot-specific transform must be calibrated by placing the
robot upright and checking that gravity is approximately `[0, 0, -1]`.

`ready` is false until a valid quaternion has been received for a warm-up window. An I2C
error invalidates freshness but does not block the control thread. The safety policy must
not treat a missing external sample as an upright sample.

## Configuration

The new configuration is intentionally explicit:

```toml
[walking_imu]
# `dynamixel` preserves the current ID-200 implementation during migration.
# `bno08x_i2c` selects the external worker.
source = "dynamixel"
bus = "/dev/i2c-pihat"
address = 0x4a
hz = 100
stale_after_ms = 100
```

The first release keeps `dynamixel` as the default. A board can select `bno08x_i2c`
only after wiring and calibration are verified. The service logs the selected source,
bus, address, product ID, sample rate and first-ready transition.

## Bring-up sequence

1. Install the hardware-I2C overlay and verify `/dev/i2c-pihat` points at I2C3.
2. Wire BNO085/BNO088 SDA/SCL, 3.3 V and ground. Pull-ups must be present; do not use
   the 5 V rail directly.
3. Confirm the address with `i2cdetect -y 3` (normally `0x4a`, sometimes `0x4b`).
4. Start with `source = "bno08x_i2c"`, torque disabled, and inspect the IMU readiness and
   stale counters in `robot.health`.
5. Check upright gravity, sign and axes before enabling a policy.
6. Compare quaternion and angular-velocity logs with the old `imu_to_dxl` on a bench.
7. Only then enable walking and tune fall thresholds. A BNO08x output rate of 100 Hz is
   recommended for a 50 Hz policy loop.

## Failure behavior

* Startup or I2C failure: the worker retries with bounded backoff and publishes no fresh
  sample. `robotd` remains responsive but does not enable walking without a ready IMU.
* Stale sample: the control loop may coast joint positions for its existing short budget,
  but it never feeds a stale sample to fall debounce or odometry.
* Invalid quaternion or impossible norm: discard the sample and increment diagnostics.
* Worker exit: report the source as unavailable and keep the motor output safe.
* `source = "dynamixel"`: preserve the existing combined read path unchanged for rollback.

## Follow-up required before production

The first adapter uses the open-source `bno080` I2C transport and quaternion-derived
gyro because that is the available I2C Rust API. Production validation must either show
that this estimate is adequate for the trained policy, or replace it with a BNO08x I2C
driver/API that enables the native gyro report. The latter is preferable for fall
detection and should be treated as a focused follow-up, not hidden behind the initial
hardware bring-up.