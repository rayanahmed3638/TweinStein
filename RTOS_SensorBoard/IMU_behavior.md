# IMU Behavior (GY-521 / MPU-6050)

Empirically verified axis mapping, signs, and scales for the IMU mounted on the racing car.

## Mounting

- GY-521 breakout mounted flat on chassis
- Silkscreen **Y axis points toward the front bumper**
- Chip Z axis points up (out of chassis)

## Body-Frame Axis Mapping (right-handed)

| Chip axis | Car direction | Physical meaning |
|-----------|---------------|------------------|
| **+X** | RIGHT side of car | lateral |
| **+Y** | FORWARD (front bumper) | longitudinal |
| **+Z** | UP (away from floor) | vertical |

Verification (static gravity tests):

| Car orientation | Expected axis = +1 g | Result |
|-----------------|----------------------|--------|
| Wheels on floor (flat) | AccelZ ≈ +1000 mg | pass |
| Nose up (on rear bumper) | AccelY ≈ +1000 mg | pass |
| Laying on LEFT side | AccelX ≈ +1000 mg | pass |

## Gyro Signs (right-hand rule, verified)

- **Counter-clockwise** rotation viewed from above → **GyroZ positive**
- Therefore: **left turn → GyroZ > 0**, **right turn → GyroZ < 0**

## Steering Sign Convention (project-wide)

**`steeringAngle > 0` means RIGHT turn.**

Consequence: raw GyroZ sign is opposite to the steering convention. When feeding yaw rate into the model or using it as a turn-rate signal, **negate GyroZ** so that "positive = right turn" throughout:

```
yaw_rate = -IMU_GyroZ    # now matches steering convention
```

## Accel Signs in Driving

With X-right, Y-forward convention:

| Event | Expected sign |
|-------|---------------|
| Forward throttle (speeding up) | AccelY > 0 |
| Braking (slowing down) | AccelY < 0 |
| Right turn (chassis accel toward right) | AccelX > 0 |
| Left turn (chassis accel toward left) | AccelX < 0 |
| Crash impact | AccelX or AccelY spike, magnitude 20000+ mg |

Note: AccelX already matches the steering convention (right turn → positive). No sign flip needed for lateral accel.

## Scale Factors (default MPU-6050 ranges)

| Quantity | Raw scale | To physical units |
|----------|-----------|-------------------|
| Accel | 16384 LSB/g | `mg = raw * 1000 / 16384` |
| Gyro  | 131 LSB/(deg/s) | `ddps = raw * 10 / 131` (0.1 deg/s) |
| Temp  | — | `T_C = raw/340 + 36.53` |

Ranges: accel ±2 g (±32768 raw), gyro ±250 deg/s (±32768 raw).

## Useful vs Unused Axes for Flat-Track Racing

**Keep (real signal):**
- `IMU_GyroZ` — yaw rate (king signal)
- `IMU_AccelY` — longitudinal (braking/throttle)
- `IMU_AccelX` — lateral (cornering g)

**Skip (near-zero on flat ground, wastes RL input dims):**
- `IMU_GyroX`, `IMU_GyroY` — pitch/roll rates
- `IMU_AccelZ` — vertical (constant gravity)

## Driver Config Summary

See `IMU.h` for tunable knobs:

- `IMU_DLPF_CFG` — digital low-pass filter setting (currently 5 = 10 Hz BW, heavy smoothing)
- `IMU_AVG_SAMPLES` — block average count per `IMU_Read` call (currently 16)

Both stack: DLPF does analog/digital low-pass in the chip, block averaging reduces residual white noise further.

## Notes / TODO

- Gyro bias calibration at boot is planned but not implemented (`IMU_Calibrate` in pseudocode, not yet in driver). Bias will drift with temperature — recal each boot while car is still.
- WHO_AM_I check assumes AD0 tied low (address 0x68). If AD0 is ever pulled high, WHO_AM_I still returns 0x68 so the check in `IMU_Init` would fail — compare against hardcoded `0x68` instead of `I2C_ADDRESS` in that case.
