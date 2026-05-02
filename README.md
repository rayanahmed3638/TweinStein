# TweinStein 🏎️

**Autonomous racing robot — 1st place, UT Austin ECE 445M / 379K Final Race.**

TweinStein is a custom two-board embedded racing platform built around a hand-written priority-preemptive RTOS on TI MSPM0 microcontrollers. It runs a multi-sensor perception stack (LiDAR, IR rangefinders, ToF, IMU) over a CAN bus, follows walls with a tuned PD controller, and overtakes opponents using a classical state machine that distinguishes moving cars from static walls via ego-motion-compensated range-rate classification.

<img width="6048" height="4024" alt="RobotRacing" src="https://github.com/user-attachments/assets/c3e351ba-bf71-405f-964d-4dbd54ea9a51" />


## Highlights

- **Custom RTOS** — preemptive scheduler with hand-written context switching, priority scheduling, semaphores, and mailboxes (`RTOS_Lab1` → `RTOS_Lab5`)
- **Two-board architecture** over CAN — `SensorBoard_v8` for perception + control, `MotorBoard_v7` for low-level actuation
- **PD wall-follower** with a dynamic lateral setpoint (`dist_ref_cur`) so high-level behaviors can steer without touching the inner loop
- **Heuristic overtaking state machine** — `S_FOLLOW → S_DETECT → S_COMMIT → S_PASS → S_REJOIN`, modulating the wall-follower's lateral setpoint to pass without a separate low-level controller
- **Ego-motion-compensated dynamic-object classifier** separates approaching walls from moving opponents using a residual `r = front_filt − front_pred` and an accumulator
- **Telemetry & calibration** — 14-column SD-card logging (`FileDumpRow`), live ST7735 display, dedicated `RobotCalib` routine that derives `V_MAX_MMPS` from front-LiDAR closure rate
- **WiFi telemetry** via ESP8266, serial logging via `log_serial.sh`

## Hardware

| Subsystem | Components |
|---|---|
| Compute | 2× TI MSPM0 (Sensor + Motor boards) |
| Perception | LDRobot LD19 360° LiDAR, TF-Luna, GP2Y0A IR, IMU (ICM-class) |
| Display | ST7735R TFT, SSD1306 OLED |
| Connectivity | CAN bus (inter-board), ESP8266 (WiFi telemetry) |
| Storage | SD card (`SDCFile`) |
| Power / Drive | Custom motor board with PWM drive + DAC |

Schematics, BOM, and pin assignments live in `BOM_v8.xlsx`, `RTOS_Pins_Assignments.xlsx`, and `Datasheets/`.

## Repository Layout

```
RTOS_Lab1..5            Incremental RTOS build: threads → kernel → priority → FS → loader
RTOS_Labs_common        Shared kernel headers/sources
RTOS_SensorBoard        Top-level firmware for the sensor+control board
RTOS_MotorBoard         Low-level motor controller firmware
SensorBoard_v8          Sensor board peripherals & drivers
MotorBoard_v7           Motor board peripherals & drivers
LD19Tester, TFLuna,     Per-sensor bring-up and test programs
GP2Y0A, ESP8266, CAN
inc/                    TM4C/MSPM0 register and helper headers
*.csv                   Logged telemetry runs (CW/CCW laps, IMU steady-state)
OVERTAKING_HANDOFF.md   Implementation notes for the overtaking state machine
```

## Build & Flash

The project targets the TI MSPM0 toolchain. Each module (`RTOS_SensorBoard`, `RTOS_MotorBoard`, etc.) is a self-contained build. The workspace file `ECE445M.theia-workspace` opens the full project in a Theia/CCS-compatible IDE.

To capture telemetry over USB serial during a run:

```bash
./log_serial.sh
```

Logs land in the repo root as `*.csv` and can be replayed against `Track_Of_Doom.csv` for offline analysis.

## How It Races

1. **Wall-follow** — the sensor board fuses LiDAR + ToF returns, computes lateral error against `dist_ref_cur`, and drives a PD loop whose output is sent over CAN to the motor board.
2. **Detect** — every tick, the classifier predicts the next front-LiDAR reading from current ego speed and compares against the filtered measurement. A sustained negative residual (something getting closer faster than the wall would) flips the state machine into `S_DETECT`.
3. **Commit & pass** — `dist_ref_cur` is ramped outward, smoothly shifting the line out and around the opponent while the PD inner loop tracks the new setpoint.
4. **Rejoin** — once the side returns clear for `T_CLEAR_MS` and the opponent is past, `dist_ref_cur` ramps back to nominal and the robot settles back onto the racing line.

See [`OVERTAKING_HANDOFF.md`](OVERTAKING_HANDOFF.md) for the full state machine, tuning knobs (`R_YAW_GAIN_Q8`, `R_THRESH_BASE`, `T_PASS_MIN`, `T_CLEAR_MS`), and calibration procedure.

## Results

🏆 **1st place — UT Austin ECE 445M / 379K Final Race**





## License

See [`license.txt`](license.txt).
