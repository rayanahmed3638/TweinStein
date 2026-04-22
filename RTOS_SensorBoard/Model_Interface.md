# Model Interface Specification

This document specifies exactly how the trained policy network interfaces with the onboard controller running on the MSPM0 sensor board. It exists so the simulation used for training can produce weights and biases that transfer directly to the robot without any additional tuning on the firmware side.

The controller is a **residual architecture**. A PD baseline controller computes an action every iteration. A small linear policy network produces a *correction* (delta) that is added to the PD action and clamped to safe limits. The simulation must train the network to output these corrections, not full actions.

---

## 1. Architecture

- **Network type:** single-layer linear regression. No hidden layers, no activations.
- **Shape:** `output[3] = W[3][13] * input[13] + b[3]`
- **Numeric format:** Q16 signed fixed point throughout. Unity = 65536. Internal accumulator uses `int64_t` to prevent overflow during multiply.
- **Parameter count:** 42 (39 weights + 3 biases).

Because the network is linear, training can treat it as a 39-weight multivariate regression or as a tiny policy network for reinforcement learning. Either is acceptable.

---

## 2. Constants (authoritative)

All constants live in `Model.h`. **Sim must use these exact values.** A shared header or a copy kept in lock-step is mandatory — any mismatch silently produces incorrect behavior on hardware.

| Constant | Value | Units | Purpose |
|---|---|---|---|
| `CAP_IR` | 305 | mm | Maximum usable IR distance; inputs above this saturate to Q16 = 65536 |
| `CAP_TFLUNA` | 1000 | mm | Maximum usable TF-Luna distance for normalization |
| `CAP_THROTTLE` | 9000 | PWM units | Maximum throttle command magnitude |
| `CAP_STEERING` | 30 | degrees | Maximum steering magnitude (±30°) |
| `CAP_ANGLE` | 64 | degrees | Wall-angle clamp magnitude for model input |
| `CAP_ANGLE_POW` | 6 | — | `log2(CAP_ANGLE)`, used to turn the angle normalize into a pure shift |
| `CAP_DELTA_STEERING` | 10 | degrees | Max magnitude of residual correction to steering |
| `CAP_DELTA_THROTTLE` | 2000 | PWM units | Max magnitude of residual correction to each throttle |
| `CAP_YAW` | 16384 | raw LSB | Yaw-rate clamp; 16384 LSB ≈ 125 deg/s at the MPU-6050 default ±250 deg/s range (131 LSB/(deg/s)) |
| `CAP_ACCEL` | 8192 | raw LSB | Accel clamp (lateral and longitudinal); 8192 LSB ≈ 0.5 g at the MPU-6050 default ±2 g range (16384 LSB/g) |

The delta caps determine **how much authority the model has over the PD controller**. Small delta caps keep the controller close to the known-safe PD baseline. Larger delta caps allow the model to override more aggressively. Start with the values above.

---

## 3. Input Vector Layout

The input vector has 13 elements in this exact order (matches `input_t` enum in `Model.h`):

| Index | Name | Source | Raw Range | Normalization | Q16 Encoding |
|---|---|---|---|---|---|
| 0 | `ir_right` | Right IR sensor, mm | [0, ≥305] | `Model_Normalize(x, CAP_IR)` | unsigned [0, 65536] |
| 1 | `ir_left` | Left IR sensor, mm | [0, ≥305] | `Model_Normalize(x, CAP_IR)` | unsigned [0, 65536] |
| 2 | `tf_left` | Left TF-Luna, mm | [0, ≥1000] | `Model_Normalize(x, CAP_TFLUNA)` | unsigned [0, 65536] |
| 3 | `tf_middle` | Front TF-Luna, mm | [0, ≥1000] | `Model_Normalize(x, CAP_TFLUNA)` | unsigned [0, 65536] |
| 4 | `tf_right` | Right TF-Luna, mm | [0, ≥1000] | `Model_Normalize(x, CAP_TFLUNA)` | unsigned [0, 65536] |
| 5 | `throttle_left_prev` | Applied left throttle, previous iter | [0, 9000] | `Model_Normalize(x, CAP_THROTTLE)` | unsigned [0, 65536] |
| 6 | `throttle_right_prev` | Applied right throttle, previous iter | [0, 9000] | `Model_Normalize(x, CAP_THROTTLE)` | unsigned [0, 65536] |
| 7 | `steering_prev` | Applied steering angle, previous iter | [-30, +30] | `Model_NormalizeSigned(x, CAP_STEERING)` | offset [0, 65536], center = 32768 |
| 8 | `angle_left` | Wall angle from left sensors, deg | clamped to [-64, +64] | `(x + 64) << 9` | offset [0, 65536], center = 32768 |
| 9 | `angle_right` | Wall angle from right sensors, deg | clamped to [-64, +64] | `(x + 64) << 9` | offset [0, 65536], center = 32768 |
| 10 | `yaw_rate` | `-IMU_GyroZ` (sign flipped so + = right turn), raw LSB | clamped to [-16384, +16384] | `Model_NormalizeSigned(x, CAP_YAW)` | offset [0, 65536], center = 32768 |
| 11 | `accel_lat` | `IMU_AccelX` (+ = chassis accel to right), raw LSB | clamped to [-8192, +8192] | `Model_NormalizeSigned(x, CAP_ACCEL)` | offset [0, 65536], center = 32768 |
| 12 | `accel_long` | `IMU_AccelY` (+ = forward accel), raw LSB | clamped to [-8192, +8192] | `Model_NormalizeSigned(x, CAP_ACCEL)` | offset [0, 65536], center = 32768 |

### Normalization formulas

**Unsigned sensor inputs** (distance, throttle):
```
out = (input << 16) / cap
if out > 65536: out = 65536          # saturate, do not wrap
```

**Signed inputs centered on zero** (steering, wall angle):
```
clamp input to [-cap, +cap]
out = ((input + cap) << 16) / (2 * cap)
```
This maps `-cap → 0`, `0 → 32768`, `+cap → 65536`.

**Wall angle (special case using power-of-two cap):**
```
clamp angle to [-64, +64]
out = (angle + 64) << 9         # equivalent to ((angle+64) << 16) / 128
```

### IMU inputs (indices 10-12)

The three IMU inputs come from the GY-521 / MPU-6050 and use **raw 16-bit LSB** from the chip, not converted physical units. Scale factors (default MPU-6050 ranges): 131 LSB/(deg/s) for gyro, 16384 LSB/g for accel. Full documentation of axis mapping, sign conventions, and scale factors lives in `IMU_behavior.md`.

Sign conventions used throughout the project:
- `steeringAngle > 0` = right turn.
- `yaw_rate > 0` = right turn. This requires **negating** `IMU_GyroZ` when reading from the driver, because the raw gyro uses right-hand-rule (left turn = positive). The firmware does this negation at the input-packing site.
- `accel_lat > 0` = chassis accelerating to the right. Matches `IMU_AccelX` directly; no sign flip.
- `accel_long > 0` = chassis accelerating forward. Matches `IMU_AccelY` directly; no sign flip.

Simulation must produce equivalent **raw-LSB** values (either by simulating the chip quantization explicitly, or by converting physical units: `raw_gyro = round(dps * 131)`, `raw_accel = round(g * 16384)`). The sim must also apply the negation on the gyro-Z channel before normalization so that the model sees a consistent "+ = right turn" convention across yaw rate and steering.

Saturation note: `Model_NormalizeSigned` clamps out-of-range inputs to `[-CAP, +CAP]` internally. Hard corners that exceed 125 deg/s yaw or 0.5 g lateral produce saturated inputs; the model cannot distinguish "at limit" from "beyond limit." Raise the caps if the log shows frequent saturation.

### Previous-action inputs

Inputs 5, 6, 7 are the **actual applied action from the previous iteration**, meaning the PD baseline plus residual delta plus any clamp. They are *not* the raw model output before clamping. The firmware writes these three inputs at the end of each control iteration, after `Model_ApplyResidual` runs.

On the first iteration, the firmware explicitly initializes `steering_prev = 32768` (center). The two throttle prev values are zero-initialized (motors off at boot).

The simulation must replicate this exactly: after applying and clamping the action, record the clamped values as the prev-inputs for the next timestep.

---

## 4. Output Vector Layout

The output vector has 3 elements in this exact order (matches `output_t` enum in `Model.h`):

| Index | Name | Encoding | Semantic |
|---|---|---|---|
| 0 | `throttle_left` | offset Q16 [0, 65536], center = 32768 | delta added to PD left throttle |
| 1 | `throttle_right` | offset Q16 [0, 65536], center = 32768 | delta added to PD right throttle |
| 2 | `steering` | offset Q16 [0, 65536], center = 32768 | delta added to PD steering |

**Output 32768 = zero delta = pure PD behavior.** This is the key property that makes untrained models safe to flash: with all weights and biases equal to zero, the model outputs zero. To make the default output 32768 (not 0), the bias vector is initialized to `{32768, 32768, 32768}`, meaning an untrained or zero-weighted model produces exactly the PD baseline action.

### Denormalization (what firmware does with outputs)

```
delta_steer = ((steering       - 32768) * CAP_DELTA_STEERING) >> 15
delta_thr_l = ((throttle_left  - 32768) * CAP_DELTA_THROTTLE) >> 15
delta_thr_r = ((throttle_right - 32768) * CAP_DELTA_THROTTLE) >> 15
```

Note the `>> 15` (not `>> 16`). Subtracting the offset halves the usable range, so one less shift reaches full `±CAP_DELTA_*`.

Final applied action:
```
steering_applied = clamp(PD_steering + delta_steer, -CAP_STEERING, +CAP_STEERING)
throttle_l_applied = clamp(PD_throttle_l + delta_thr_l, 0, CAP_THROTTLE)
throttle_r_applied = clamp(PD_throttle_r + delta_thr_r, 0, CAP_THROTTLE)
```

---

## 5. Inference

Pure matrix multiply plus bias. Pseudocode:

```
for r in 0..NUM_OUTPUTS:
  Model_Outputs[r] = 0
  for c in 0..NUM_INPUTS:
    Model_Outputs[r] += fixed_mul(Model_Weights[r][c], Model_Inputs[c])
  Model_Outputs[r] += Model_Bias[r]

where fixed_mul(x, y) = ((int64_t)x * y) >> 16
```

Weights and biases are `int32_t` (signed). Inputs are effectively unsigned but stored as `int32_t`. Output accumulator is `int32_t`; individual multiplies use `int64_t` intermediate to prevent Q16×Q16 overflow.

No clamping is applied to the raw output before denormalization. If the linear math produces values outside `[0, 65536]`, the final clamp in `Model_ApplyResidual` catches them. This keeps inference lightweight.

---

## 6. PD Baseline (what sim must replicate before training)

The residual model only learns corrections on top of PD. The simulation must implement the exact PD controller used on-device, otherwise residuals learned in sim will not generalize to the robot. The controller is in `RTOS_SensorBoard.c:Robot()`.

### Step-by-step

Given sensor readings `d_ir`, `ld_ir` (right/left IR, mm), `d2`, `ld2` (right/left TF-Luna, mm), `front` (front TF-Luna, mm):

```
# Wall geometry
angle   = arctan((d_ir*1414  - d2*1000)  / (224 + d2))  - 5        # right-side wall angle, deg
L_angle = arctan((ld_ir*1414 - ld2*1000) / (224 + ld2))            # left-side wall angle, deg

realDist   = d_ir  * cos(angle)   / 1000        # perpendicular right-wall distance
L_realDist = ld_ir * cos(L_angle) / 1000        # perpendicular left-wall distance

# Distance PD
e_d = realDist - L_realDist - 0                                   # dist_ref = 0
intend_angle = (kp_d * e_d) / 10 + (kd_d * (e_d - prevError)) / 10
prevError = e_d

# Angle PD
e_a = intend_angle - angle
steering = (e_a * kp_a) / 10 + ((e_a - prevE_A) * kd_a) / 10
prevE_A = e_a

# Base throttle
throttle_base = 9000

# Corner override
if front < 600:
  throttle_base -= 2000
  urgency = (600 - front) >> 4
  steering = -urgency if (ld2 > d2) else +urgency

# Clamp steering
steering = clamp(steering, -30, +30)

# Differential steering
throttle_l = throttle_base
throttle_r = throttle_base
if steering <= -15: throttle_r -= 2000
elif steering >= 15: throttle_l -= 2000
```

### PD gain constants

| Constant | Value |
|---|---|
| `kp_d` | 1 |
| `kd_d` | 2 |
| `kp_a` | 5 |
| `kd_a` | 2 |
| `angle_ref` | 5 |
| `dist_ref` | 0 |

All PD math uses integer division by 10 in two places.

### Integer arithmetic

The firmware does everything in signed integer math. The sim may use floats internally, but the PD path should behave identically if arithmetic precision is sufficient. Integer truncation in the real PD means tiny error terms can round to zero — if the sim is too precise, residual training may chase rounding artifacts that do not exist on the robot.

---

## 7. Full Simulation Control Loop (per timestep)

```
# 1. Read simulated sensors
(d_ir, ld_ir, d2, ld2, front) = sim_sensors()
(raw_gyro_z, raw_accel_x, raw_accel_y) = sim_imu_raw_lsb()     # see Section 3 for scale

# 2. Run PD baseline (see Section 6)
(pd_throttle_l, pd_throttle_r, pd_steering) = pd_controller(sensors, prev_pd_state)

# 3. Build model input vector (see Section 3)
inputs[0]  = normalize(d_ir,  CAP_IR)
inputs[1]  = normalize(ld_ir, CAP_IR)
inputs[2]  = normalize(ld2,   CAP_TFLUNA)
inputs[3]  = normalize(front, CAP_TFLUNA)
inputs[4]  = normalize(d2,    CAP_TFLUNA)
inputs[5]  = prev_applied_throttle_l_normalized      # from previous timestep
inputs[6]  = prev_applied_throttle_r_normalized
inputs[7]  = prev_applied_steering_normalized
inputs[8]  = encode_angle(L_angle)
inputs[9]  = encode_angle(angle)
inputs[10] = normalize_signed(-raw_gyro_z, CAP_YAW)            # sign flip: + = right turn
inputs[11] = normalize_signed( raw_accel_x, CAP_ACCEL)          # + = right
inputs[12] = normalize_signed( raw_accel_y, CAP_ACCEL)          # + = forward

# 4. Run inference
raw_outputs = W @ inputs + b

# 5. Compute deltas (see Section 4)
delta_steer = ((raw_outputs[2] - 32768) * CAP_DELTA_STEERING) >> 15
delta_thr_l = ((raw_outputs[0] - 32768) * CAP_DELTA_THROTTLE) >> 15
delta_thr_r = ((raw_outputs[1] - 32768) * CAP_DELTA_THROTTLE) >> 15

# 6. Apply and clamp
applied_steering   = clamp(pd_steering   + delta_steer, -CAP_STEERING, +CAP_STEERING)
applied_throttle_l = clamp(pd_throttle_l + delta_thr_l, 0, CAP_THROTTLE)
applied_throttle_r = clamp(pd_throttle_r + delta_thr_r, 0, CAP_THROTTLE)

# 7. Update prev-inputs for next timestep
prev_applied_throttle_l_normalized = normalize(applied_throttle_l, CAP_THROTTLE)
prev_applied_throttle_r_normalized = normalize(applied_throttle_r, CAP_THROTTLE)
prev_applied_steering_normalized   = normalize_signed(applied_steering, CAP_STEERING)

# 8. Step physics with (applied_throttle_l, applied_throttle_r, applied_steering)
```

### Initial conditions

At the start of each simulated episode:
- `prev_applied_throttle_l_normalized = 0`
- `prev_applied_throttle_r_normalized = 0`
- `prev_applied_steering_normalized = 32768`
- PD state (`prevError`, `prevE_A`) = 0

### Control rate

The firmware loop runs at approximately **12.5 Hz** (one control iteration per sensor update cycle). Simulation timestep should target the same rate so trained weights assume the correct latency and discretization.

---

## 8. Captured Log Format (`robot0`)

The robot writes a CSV of every control iteration to the onboard SD card under the filename **`robot0`** (no extension — the custom `eFile` filesystem stores raw blocks, not FAT). The file is rewritten from scratch at the start of every run (`eFile_Delete` then `eFile_Create`).

### Header line

```
time_ms,ir_r,ir_l,tf_r,tf_l,tf_front,throttle_l,throttle_r,steering,gyro_z,accel_x,accel_y
```

### Column definitions

| Column | Units | Notes |
|---|---|---|
| `time_ms` | ms since `OS_ClearMsTime()` at Robot() start | monotonic per run |
| `ir_r` | mm | right IR reading (`d_ir` in firmware, already doubled from raw `Distance`) |
| `ir_l` | mm | left IR reading (`ld_ir`, already doubled from raw `L_Distance`) |
| `tf_r` | mm | right TF-Luna (`d2`, 8-sample mean unless `USE_MEDIAN_FILTER`) |
| `tf_l` | mm | left TF-Luna (`ld2`, same filter) |
| `tf_front` | mm | front TF-Luna (`FrontDist`) |
| `throttle_l` | PWM units, [0, 9000] | applied left throttle sent to CAN |
| `throttle_r` | PWM units, [0, 9000] | applied right throttle sent to CAN |
| `steering` | degrees, [-30, +30] | applied steering angle sent to CAN, signed decimal |
| `gyro_z` | raw LSB, signed | `IMU_GyroZ` as read from the chip (NOT sign-flipped). 131 LSB per deg/s. Negate at parse time to match the project's "+ = right turn" convention. |
| `accel_x` | raw LSB, signed | `IMU_AccelX` as read. 16384 LSB per g. + = chassis accel to right (matches steering sign). |
| `accel_y` | raw LSB, signed | `IMU_AccelY` as read. 16384 LSB per g. + = forward accel (speeding up). |

All integer decimals. One row per control iteration (~12.5 Hz). No trailing comma. Newline is `\n`. IMU values are snapshotted from the IMU globals at the moment of logging; the `IMU_Task` thread updates them at ~50 Hz, so consecutive rows may repeat identical IMU samples.

### Capture workflow (host side)

1. Flash the sensor-board firmware with `Robot()` as the active main and logging enabled (uncomment the `FileDumpRow` block in `Robot()`).
2. Drive a lap. Firmware stops logging on disk-full; stop conditions are handled in-loop.
3. Reflash with `DumpRobotFileMain()` as the active main (trampoline in `main()`).
4. On the host: `./log_serial.sh robot0.csv` — the script auto-detects `/dev/ttyACM0/1` or `/dev/ttyUSB0/1`, configures 115200 8N1 raw, and streams UART to the file.
5. Press `S2` (PB21) on the LaunchPad to trigger the dump.
6. Stop the host capture with Ctrl+C once streaming ends.
7. Open `robot0.csv` in a text editor; delete any non-CSV preamble lines (e.g. the "Ready. Press S2..." prompt) if present, then load in Excel or pandas.

### Simulation uses of the log

The log is the ground truth for aligning the sim to the real robot. Three uses:

1. **Sensor noise fit.** Compute statistics (variance, autocorrelation, bias) on each sensor column during straight-line segments. Inject matching noise into the sim so trained residuals do not chase noise patterns that differ between sim and hardware.
2. **PD replay validation.** Feed `(ir_r, ir_l, tf_r, tf_l, tf_front)` through the sim's PD controller (Section 6). Compare the sim's computed `throttle_l`, `throttle_r`, `steering` against the logged columns. They should match up to integer rounding. Any systematic divergence indicates a sim-side implementation bug in the PD path that must be fixed **before** any RL training runs.
3. **Behavioral cloning warm start.** Feed the log through the sim pipeline, train the residual network to predict near-zero deltas (because PD alone produced the logged actions). This gives a safe starting point for RL fine-tuning with the correct sim plumbing verified.

All three uses require the sim's PD controller, normalization, and sensor saturation to match `Model_Interface.md` exactly before the log data becomes meaningful.

---

## 9. Training Recommendations

### Objective

The model must learn small corrections that improve lap performance beyond the PD baseline. Suggested reward components:

- **Positive:** forward progress along track centerline; lap completion.
- **Negative:** wall contact; excessive yaw rate; large delta magnitude (discourages the model from fighting PD when PD is already good).

Apply a small L2 penalty on weights to keep the residual conservative. A large magnitude residual means the model is overriding PD instead of correcting it — usually a sign of training instability or misaligned sim physics.

### Method choices

The network is small enough (33 parameters) that several methods work:

1. **Evolution strategies** — spawn N weight vectors, evaluate each on one or more tracks, mutate the best, iterate. Gradient-free, handles the clamp non-differentiability, works well at this scale.
2. **REINFORCE / PPO** — standard policy gradient; treat the linear model as a deterministic policy with Gaussian exploration noise added to outputs during training.
3. **Behavioral cloning from logs** — record laps with the PD-only controller, then train the residual to predict zero everywhere (producing the PD behavior). This is a cheap sanity check: a correctly trained residual on PD-optimal data should produce near-zero deltas. Use the robot-log CSV captured via the `DumpRobotFile` path.
4. **Residual DDPG** — works for continuous actions; requires care to prevent large residuals.

### Sim fidelity requirements

Items the sim must model for trained weights to transfer:
- PD controller matching the pseudocode in Section 6 bit-for-bit where integer arithmetic is visible.
- Sensor saturation at the cap values (IR >305mm and TF-Luna >1000mm produce Q16 = 65536).
- Corner override triggered at `front < 600`, with the exact `urgency = (600-front) >> 4` formula.
- Differential steering thresholds at ±15°.
- One-timestep latency between action and next observation (prev-inputs are from the previous iteration).
- A measurement-noise model on the sensor values. Use the captured robot log to fit noise statistics.

---

## 10. Weight Export

After training, export weights as C array literals in Q16:

```
int32_t q_weight = round(float_weight * 65536)
int32_t q_bias   = round(float_bias   * 65536)
```

The final artifact is a block of C that replaces the current initializers in `Model.c`:

```c
const fixed_t Model_Weights[NUM_OUTPUTS][NUM_INPUTS] = {
  { w_00, w_01, w_02, w_03, w_04, w_05, w_06, w_07, w_08, w_09, w_0A, w_0B, w_0C },  // throttle_left
  { w_10, w_11, w_12, w_13, w_14, w_15, w_16, w_17, w_18, w_19, w_1A, w_1B, w_1C },  // throttle_right
  { w_20, w_21, w_22, w_23, w_24, w_25, w_26, w_27, w_28, w_29, w_2A, w_2B, w_2C },  // steering
};

const fixed_t Model_Bias[NUM_OUTPUTS] = { b_0, b_1, b_2 };
```

Column order within each row must match the `input_t` enum order listed in Section 3. Row order must match the `output_t` enum order listed in Section 4.

### Numeric range checks before export

- Each weight's absolute value should stay well below `2^15` in Q16 (i.e., raw float magnitude below ~0.5), or intermediate `fixed_mul` products can be large enough that the accumulator approaches `int32_t` limits when summed across 13 inputs. If weights grow larger, enforce an L2 penalty during training.
- Bias magnitudes typically stay within `[0, 65536]` (the output range). Values far outside this range indicate the model is trying to shift the nominal output far from center — usually a training issue.

### Validation

Before deploying to hardware:
1. Replay the robot's captured log CSV through the simulated pipeline and confirm the sim produces the same PD outputs as were logged on the robot (up to rounding).
2. Run the trained model in the sim for full laps; verify no crashes, no throttle saturation, and modest delta magnitudes.
3. Flash, start in low-risk mode (e.g., reduced `CAP_DELTA_*` for the first on-track run) and scale up if behavior is stable.
