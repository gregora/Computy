# Firmware specifications

## Operation modes

| Mode | Name        | Description                                         |
|------|-------------|-----------------------------------------------------|
| 0    | Manual      | Direct output to PWM signals                        |
| 1    | Take-off    | Control angular velocities                          |
| 2    | Fly-by-wire | Assume 10 degree pitch up attitude                  |
| 255  | Recovery    | Mode 0, but will only exit when all switches are up |

