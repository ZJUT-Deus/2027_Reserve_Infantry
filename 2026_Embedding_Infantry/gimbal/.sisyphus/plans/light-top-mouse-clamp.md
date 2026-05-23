# Light TOP mouse yaw clamp

## Goal

Reduce yaw jerk/snap during small-gyro manual mouse sweeps with the smallest safe code change.

## Scope

- Modify only `user_code/module/gimbal.h` and `user_code/module/gimbal.cpp`
- Add one tunable limit constant for TOP small-gyro manual mouse yaw increment
- Clamp only the mouse yaw contribution in the manual branch of `Gimbal::gimbal_rc_to_control_angle()`

## Constraints

- Keep RC yaw contribution unchanged
- Keep FREE mode behavior unchanged
- Keep vision tracking takeover branch unchanged
- Do not change PID gains, output logic, or chassis logic
- Apply clamp only when `gimbal_mode == GIMBAL_TOP`, `chassis.chassis_behaviour_mode == CHASSIS_TOP`, and `top_switch == TRUE`

## Validation

- Re-read patched code for scope correctness
- Run LSP diagnostics on modified files if available
- Check for project file / build entry presence and report whether build can be run in current environment
