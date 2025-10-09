# Firmware specifications

## Operation modes

| Mode | Name        | Description                                |
|------|-------------|--------------------------------------------|
| 0    | Manual      | Direct output to PWM signals               |
| 1    | Take-off    | Assume 10 degree pitch up attitude         |
| 2    | Fly-by-wire | Control angular velocities                 |
| 3    | Automatic   | Fly the mission autonomously               |
| 255  | Recovery    | Mode 0, only exit when all switches are up |

