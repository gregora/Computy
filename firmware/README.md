# Firmware specifications

## Operation modes

| Mode | Name        | Description                                |
|------|-------------|--------------------------------------------|
| 0    | Manual      | Direct output to PWM signals               |
| 1    | Take-off    | Assume 10 degree pitch up attitude         |
| 2    | Fly-by-wire | Control angular velocities                 |
| 3    | Automatic   | Fly the mission autonomously               |
| 255  | Recovery    | Mode 0, only exit when all switches are up |



## Parameters

| Id  | Name        | Description                                |
|-----|-------------|--------------------------------------------|
|   0 | kP_roll     | Roll kP value for inner PID loop           |
|   1 | kI_roll     | Roll kI value for inner PID loop (unused)  |
|   2 | kD_roll     | Roll kD value for inner PID loop           |
|   3 | kP_pitch    | Pitch kP value for inner PID loop          |
|   4 | kI_pitch    | Pitch kI value for inner PID loop (unused) |
|   5 | kD_pitch    | Pitch kD value for inner PID loop          |
| 255 | light_T     | Light on/off time                          |

