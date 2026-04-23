# Handoff: Classical-Control Overtaking Implementation

## Context & Goal
We implemented a classical, heuristic-only overtaking state machine for the robot. This feature runs alongside the experimental residual-RL model and executes overtakes by dynamically adjusting the lateral setpoint (`dist_ref_cur`) of the existing PD wall-follower.

## Files Modified
- `RTOS_SensorBoard/RTOS_SensorBoard.c`

## Key Implementations
1. **Overtaking State Machine (`overtake_state_t`)**: 
   - States: `S_FOLLOW`, `S_DETECT`, `S_COMMIT`, `S_PASS`, `S_REJOIN`.
   - Modulates `dist_ref_cur` to smoothly shift the robot out of line and past an opponent without needing a new low-level controller.
2. **Dynamic Object Classifier**: 
   - Uses an ego-motion-compensated range-rate classifier to differentiate between static walls (which approach at ego speed) and moving cars.
   - Calculates a residual (`r = front_filt - front_pred`) and accumulates it (`r_acc`).
3. **Model Gating**: 
   - `Model_ApplyResidual(&throttle_l, &throttle_r, &steeringAngle)` is now strictly gated to only apply when `ot_state == S_FOLLOW`. It won't interfere with the passing logic.
4. **Logging (`FileDumpRow`)**: 
   - Increased `NUMCOLS` to 14. 
   - Now logs `ot_state` and `dist_ref_cur` alongside standard telemetry for post-run analysis.
5. **Calibration Function (`RobotCalib`)**: 
   - Added a dedicated routine to calibrate the throttle-to-velocity mapping (`V_MAX_MMPS`).
   - Drives perfectly straight (`steeringAngle = 3100;`) at `CALIB_THROTTLE`.
   - Calculates `-d(front)/dt` over the period where `50 <= front <= 1000` (after a 500ms startup delay) and outputs the average velocity to the ST7735 display.

## Current Status
- The code is fully integrated into `RTOS_SensorBoard.c` but **requires empirical testing and tuning**. 

## Next Steps for the Next Agent/User
1. **Run `RobotCalib()`**: Hook up `RobotCalib` to `S2Push()` or run it directly. Let the robot drive at a wall/target and observe the reported "Avg Vel". 
2. **Update Parameters**: Use the calibration result to update `V_MAX_MMPS` (currently `2000` as a placeholder) at the top of `RTOS_SensorBoard.c`. 
3. **Bench Testing**: Verify the state machine logic transitions correctly when waiving an obstacle in front of the sensors.
4. **Solo On-Track Tuning**: Drive the robot alone on the track. If the classifier fires (transitions to `DETECT` or `COMMIT`) on apexes/corners, tune `R_YAW_GAIN_Q8` and `R_THRESH_BASE` until the false positives disappear.
5. **Opponent Testing**: Test passing on straights using stationary and moving opponents. Watch for edge/corner cases or premature rejoins (`T_PASS_MIN`, `T_CLEAR_MS`).