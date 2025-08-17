# Firmware specifications

## Operation modes

| Mode | Name          | Description                                                     |
|------|---------------|-----------------------------------------------------------------|
| 0    | Normal mode   | Direct output to PWM signals                                    |
| 1    | Fly-by-wire   | Control angular velocities                                      |
| 2    | Take-off mode | Assume 10 degree pitch up attitude                              |
| 255  | Recovery      | Mode 0, but will only exit when all switches are in up position |

