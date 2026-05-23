# Shoot fire-control redesign plan

## Goal

Replace the current encoder-based local shot/heat logic with a friction-wheel current pulse detector, while preserving the existing `shoot_mode` main state machine and `trigger_anti_jam_state` anti-jam state machine.

## Files in scope

- `user_code/module/Shoot.h`
- `user_code/module/Shoot.cpp`
- `D:/桌面/内容制作/obsidian库/spike的向日葵/demo/Farm/草稿本/射击逻辑.md`
- `D:/桌面/内容制作/obsidian库/spike的向日葵/demo/Farm/草稿本/射击逻辑示意图.md`

## Keep unchanged

- `Shoot::set_mode()` behavior and fire command sources
- `Shoot::trigger_motor_turn_back()` anti-jam execution flow
- `Shoot::trigger_motor_blocked()` jam detection semantics
- Trigger/fraction wheel PID ownership and update order in `shoot_task`

## Remove / replace

- Remove encoder-based shot detection from `Shoot::cooling_ctrl()`
- Remove `heat_valid_fire_window()` and `heat_get_forward_trigger_ecd_delta()` helpers
- Remove old sync-tail, encoder accumulation, and delayed shot queue state

## New detector design

### Detector states

- `FRIC_SHOT_STOPPED`: friction wheels are off or not yet ready for detection
- `FRIC_SHOT_READY`: friction wheels are at target speed and detector is armed
- `FRIC_SHOT_SUSPECT`: current pulse is above threshold and must stay long enough to confirm
- `FRIC_SHOT_REFRACTORY`: a shot was confirmed; hold off re-triggering for a short cooldown

### Input signals

- `abs(fric_motor_left.motor_measure->given_current)`
- `abs(fric_motor_right.motor_measure->given_current)`
- friction wheel measured speed and set speed
- `shoot_mode`
- `trigger_anti_jam_state`
- `trigger_forward_speed_set`

### Derived signals

- `left_detect_current = abs(left_current) * 100`
- `right_detect_current = abs(right_current) * 100`
- `shot_current_raw = max(left_detect_current, right_detect_current)`
- `shot_current_fast`: fast EMA / low-pass
- `shot_current_slow`: slow EMA / baseline
- `shot_current_contrast = max(0, shot_current_fast - shot_current_slow)`

### Arm conditions

Detection can run only when all are true:

- `shoot_mode != SHOOT_STOP`
- both friction feedback pointers are valid
- friction set speed is non-zero
- both friction wheel measured speeds are near their commanded speeds for a dwell time

### Confirm conditions

- In `READY`, if contrast rises above a high threshold, enter `SUSPECT`
- In `SUSPECT`, if contrast falls below a low threshold early, return to `READY`
- If `SUSPECT` holds for `confirm_ticks`, emit exactly one shot and enter `REFRACTORY`
- In `REFRACTORY`, wait `refractory_ticks`, then return to `READY`

### Heat logic

- Cool `local_heat` every control cycle
- On each confirmed shot, `local_heat += SHOOT_HEAT_PER_BULLET`
- Keep dual-threshold block/rearm hysteresis
- When blocked, zero the trigger forward command only in `SHOOT_CONTINUE_BULLET` and anti-jam `IDLE`

## Verification plan

1. Run LSP diagnostics on modified files
2. Run repo build / compile command if available
3. Ensure docs match the new friction-current design instead of the deleted encoder design
4. Consult Oracle on whether the final detector thresholds/state split have hidden failure risks
