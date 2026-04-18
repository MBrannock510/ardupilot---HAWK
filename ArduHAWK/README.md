# ArduHAWK

This folder is the repo-root workspace for HAWK-specific notes and handoff material.
Build-critical ArduPilot source files remain in their normal upstream locations under
`libraries/`, `ArduCopter/`, and `Tools/`.

## HAWK Team - Wiring

Target board: `TMotorH743` / T-Motor H7 Mini

Real-flight actuator layout:

- `SERVO1` / native `S1` / `M1` -> ESC signal for HAWK motor 1
- `SERVO2` / native `S2` / `M2` -> ESC signal for HAWK motor 2
- `SERVO3` / native `S3` / `M3` -> ESC signal for HAWK motor 3
- `SERVO4` / native `S4` / `M4` -> blade pivot continuous-rotation servo
- `SERVO9` / native `S9` / `M9` -> LED only

PCA9685 servo expander:

- Bus: `I2C1`
- FC pins: `PB6=SCL`, `PB7=SDA`
- I2C address: `0x40`
- Logic power: FC `5V` -> PCA9685 `VCC`
- Ground: FC `GND` -> PCA9685 `GND`
- Servo rail: external regulated servo supply -> PCA9685 `V+`
- Output enable: keep `OE` low / grounded
- Common ground is required between FC, PCA9685, servo power rail, ESCs, GNSS, and radios

PCA9685 output map:

- `SERVO5` -> PCA9685 ch0 -> landing leg deploy left
- `SERVO6` -> PCA9685 ch1 -> landing leg deploy right
- `SERVO7` -> PCA9685 ch2 -> tail deploy left
- `SERVO8` -> PCA9685 ch3 -> tail deploy right
- `SERVO10` -> PCA9685 ch4 -> tail actuation left
- `SERVO11` -> PCA9685 ch5 -> tail actuation right
- `SERVO12` -> PCA9685 ch6 -> spare
- `SERVO13-21` -> PCA9685 ch7-15 -> spare expansion

Serial / peripheral layout:

- `UART3` / `TX3`,`RX3` -> telemetry transceiver
- `UART5` / `TX5`,`RX5` -> GNSS
- `UART6` / `TX6`,`RX6` -> RC receiver / RC transceiver if used
- `USART1 RX` -> ESC telemetry input if available from the ESC stack

I2C shared bus devices:

- onboard barometer `DPS310` at `0x76`
- AS5600 rotor encoder at `0x36`
- PCA9685 servo expander at `0x40`
- optional external compass also shares `I2C1`

Firmware defaults currently embedded for this setup:

- `FRAME_CLASS=18` (`HAWK`)
- `FRAME_TYPE=12`
- `SERIAL3_PROTOCOL=2`, `SERIAL3_BAUD=57`
- `SERIAL5_PROTOCOL=5`, `SERIAL5_BAUD=115`
- `SERIAL6_PROTOCOL=23`
- `SERVO1_FUNCTION=33`
- `SERVO2_FUNCTION=34`
- `SERVO3_FUNCTION=35`
- `SERVO4_FUNCTION=41`
- `SERVO4_MIN/TRIM/MAX=1000/1500/2000`
- `SERVO9_FUNCTION=120`

Relevant source files:

- `libraries/AP_HAL_ChibiOS/hwdef/TMotorH743/hwdef.dat`
- `libraries/AP_HAL_ChibiOS/hwdef/TMotorH743/defaults.parm`
- `libraries/AP_HAL_ChibiOS/RCOutput.cpp`
- `libraries/AP_HAL_ChibiOS/RCOutput_PCA9685.cpp`
- `libraries/AP_Motors/AP_MotorsHawk.cpp`
- `libraries/AP_HawkEncoder/AP_HawkEncoder.h`

## CODEX - Where We Left Off

Status:

- HAWK SITL-only frame content was removed from `Tools/autotest`
- HAWK now uses one AS5600 encoder only
- Rotor timing estimation is active and the three motor phases are derived by `120` degree offsets from that single encoder
- Native outputs are hardcoded for motors on `S1-S3` and the pivot servo on `S4`
- ChibiOS RC output was extended so the `TMotorH743` can drive extra logical servo outputs through a PCA9685 on `I2C1`
- Board defaults now reserve PCA9685-backed channels for landing gear, tail deploy, and tail actuation hardware
- `./waf copter` for `TMotorH743` builds successfully with these changes

What was intentionally not moved into `ArduHAWK`:

- ArduPilot source files needed by the build system
- board hwdef files under `libraries/AP_HAL_ChibiOS/hwdef/TMotorH743`
- motor and encoder implementation under `libraries/`
- Copter / Tools integration files

Open items before any real flight attempt:

- bench-verify all native outputs and every PCA9685 output with actual hardware connected
- verify the shared `I2C1` bus stays clean with baro + AS5600 + PCA9685 present together
- confirm the telemetry transceiver type and any required `SERIAL3_OPTIONS`
- confirm the RC receiver / RC transceiver protocol on `UART6`
- define the actual control semantics for tail deployment and tail actuation rather than leaving them as reserved servo channels
- verify servo rail power capacity separately from FC logic power

Extra notes moved here:

- [Vertical_Control_Notes.md](/home/thena/ardupilot/ArduHAWK/Vertical_Control_Notes.md)
