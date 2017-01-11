# duckietown_arduino

Arduino based controller for Dukietown robots. This library is based on [serial_dxl](https://github.com/rorromr/serial_dxl).

Microcontroller: ATmega 168 3.3V 8MHz
Board: Arduino Pro Mini
H Bridge: Double L9110

| Pin |    Function   |
|-----|---------------|
| 5   | PWM Ch1 A-1B  |
| 6   | PWM Ch2 B-1B  |
| 11  | PWM Ch1 A-1A  |
| 12  | PWM Ch2 B-1A  |

Registros:

| Address |   Type   | Size | Function                   |
|---------|----------|------|----------------------------|
| 6       | uint8_t  | 1    | LED (0: off, 1: on)        |
| 7       | int16_t  | 2    | PWM channel 1 (-255 to 255)|
| 9       | int16_t  | 2    | PWM channel 1 (-255 to 255)|

