# Hoverboard Wheel Calibration With ODrive

## Overview
This document records the exact ODrive hoverboard wheel setup, calibration, debugging, and motion tests completed on the laptop.

Hardware used:
- ODrive v3.6
- Firmware 0.5.1
- Hoverboard wheels with hall sensors
- Laptop USB connection to ODrive
- Axis mapping:
  - `axis0` -> `M0`
  - `axis1` -> `M1`

Wheel and encoder details used:
- Wheel diameter: `6.5 in` (`0.1651 m`)
- Wheel circumference: `0.518 m`
- Hall encoder CPR: `90`
- Pole pairs: `15`

Python environment used:
- Virtual environment: `/home/asif/odrive-venv`

ODrive device observed:
- USB device: `Generic ODrive Robotics ODrive v3`
- Serial device: `/dev/ttyACM0`
- Serial number observed: `35588931532107`

## ODrive configuration used for both wheels
The same hoverboard hall-sensor configuration was applied to both axes.

Configuration values:
- `pole_pairs = 15`
- `motor_type = 0`
- `calibration_current = 8`
- `resistance_calib_max_voltage = 4`
- `encoder.mode = 1`
- `encoder.cpr = 90`
- `encoder.bandwidth = 100`

Equivalent Python configuration:

```python
axis.motor.config.pole_pairs = 15
axis.motor.config.motor_type = 0
axis.motor.config.calibration_current = 8
axis.motor.config.resistance_calib_max_voltage = 4
axis.encoder.config.mode = 1
axis.encoder.config.cpr = 90
axis.encoder.config.bandwidth = 100
```

## Calibration workflow used
For each axis, the workflow used was:

1. Connect to ODrive with Python `odrive`
2. Select the target axis
3. Clear existing errors
4. Apply the hoverboard configuration
5. Request full calibration sequence with state `3`
6. Wait for the axis to return to idle state `1`
7. Verify:
   - `axis.error == 0`
   - `motor.error == 0`
   - `encoder.error == 0`
   - `controller.error == 0`
   - `motor.is_calibrated == True`
   - `encoder.is_ready == True`

Calibration states observed during successful runs:
- `4`
- `7`
- `1`

## Successful M0 calibration result
`M0` was calibrated successfully on `axis0`.

Verified result:
- `axis0_error = 0`
- `motor_error = 0`
- `encoder_error = 0`
- `controller_error = 0`
- `motor_is_calibrated = True`
- `encoder_is_ready = True`

## Successful M1 calibration result
`M1` initially failed because of hall wiring issues, but after rewiring it calibrated successfully on `axis1`.

Final verified result:
- `axis1_error = 0`
- `motor_error = 0`
- `encoder_error = 0`
- `controller_error = 0`
- `motor_is_calibrated = True`
- `encoder_is_ready = True`

## Important M1 failure that happened before rewiring
Before the `M1` hall wiring was corrected, `axis1` repeatedly failed with:
- `AXIS_ERROR_ENCODER_FAILED = 256`
- `ENCODER_ERROR_ILLEGAL_HALL_STATE = 16`

Meaning:
- The hall sensor signals on `M1` were invalid.
- The software configuration was not the root cause.
- The problem was fixed only after changing the pin wiring.

Likely causes considered during debugging:
- hall connector loose
- incorrect `Hall A/B/C` order
- missing `5V` or `GND`
- motor phase wires and hall wires from different wheels
- damaged hall wire or bad pin contact

## Important M0 reconnect issue that happened once
After disconnecting and reconnecting the ODrive, `M0` temporarily showed:
- `AXIS_ERROR_ENCODER_FAILED`
- `AXIS_ERROR_MOTOR_FAILED`
- `MOTOR_ERROR_CURRENT_LIMIT_VIOLATION`
- `ENCODER_ERROR_ILLEGAL_HALL_STATE`

That issue was cleared by re-running the `M0` calibration with the correct configuration.

## Final verified dual-calibrated state
At the end, both axes were verified healthy at the same time.

Final readback:
- `axis0_current_state = 1`
- `axis0_error = 0`
- `axis0_motor_error = 0`
- `axis0_encoder_error = 0`
- `axis0_controller_error = 0`
- `axis0_motor_is_calibrated = True`
- `axis0_encoder_is_ready = True`
- `axis0_encoder_mode = 1`
- `axis0_encoder_cpr = 90`

- `axis1_current_state = 1`
- `axis1_error = 0`
- `axis1_motor_error = 0`
- `axis1_encoder_error = 0`
- `axis1_controller_error = 0`
- `axis1_motor_is_calibrated = True`
- `axis1_encoder_is_ready = True`
- `axis1_encoder_mode = 1`
- `axis1_encoder_cpr = 90`

## Low-speed motion control used
Velocity control was used for testing.

Control mode values:
- `2` = velocity control
- `8` = closed loop control

Example used to start an axis slowly:

```python
axis.clear_errors()
axis.controller.config.control_mode = 2
axis.requested_state = 8
axis.controller.input_vel = 0.10
```

Example used to stop an axis:

```python
axis.controller.input_vel = 0
```

## M0 low-speed motion test
`M0` was tested successfully at low speed.

Command used conceptually:
- `axis0.controller.input_vel = 0.10`

Verified behavior:
- `axis0` entered closed-loop state `8`
- `axis0` remained error-free
- encoder position increased over time
- wheel rotated steadily at low speed

## M1 low-speed readiness
`M1` was not started until it calibrated cleanly.
After rewiring and calibration, it became healthy and ready for motion.

## Dual-wheel opposite-direction test
Both wheels were run at low speed in opposite directions.

Commands used conceptually:

```python
odrv.axis0.controller.input_vel = 0.10
odrv.axis1.controller.input_vel = -0.10
```

Verified behavior:
- `axis0_state = 8`
- `axis1_state = 8`
- `axis0_error = 0`
- `axis1_error = 0`
- `M0` position increased
- `M1` position decreased

This confirmed opposite-direction motion.

## Live rotation values observed while both wheels were moving
Live rotations were printed while both wheels were running.

Observed trend:
- `M0` rotations increased from about `9.688612` to `9.955914`
- `M1` rotations decreased from about `-8.699889` to `-8.967467`

This confirmed:
- `M0` forward direction
- `M1` reverse direction

## Distance and step conversion formulas used
Distance formula:

```python
wheel_circumference = 0.518
distance_m = rotations * wheel_circumference
```

Step formula:

```python
steps_per_rotation = 100
steps = rotations * steps_per_rotation
```

## Recommended safe testing rules
- Keep wheels lifted/free during first tests
- Use low speed first
- Clear errors before recalibration
- Do not force motion on an axis with encoder or hall faults
- If `ENCODER_ERROR_ILLEGAL_HALL_STATE` appears, inspect hall wiring first
- Use encoder position as the source of truth for rotation

## Python snippets used in practice
### Find ODrive and inspect axis
```python
import odrive
odrv = odrive.find_any(timeout=12)
axis = odrv.axis0
print(axis.error)
print(axis.motor.error)
print(axis.encoder.error)
print(axis.encoder.pos_estimate)
print(axis.encoder.vel_estimate)
```

### Calibrate one axis
```python
import time
import odrive

odrv = odrive.find_any(timeout=12)
axis = odrv.axis0
axis.clear_errors()
axis.motor.config.pole_pairs = 15
axis.motor.config.motor_type = 0
axis.motor.config.calibration_current = 8
axis.motor.config.resistance_calib_max_voltage = 4
axis.encoder.config.mode = 1
axis.encoder.config.cpr = 90
axis.encoder.config.bandwidth = 100
axis.requested_state = 3

start = time.time()
while time.time() - start < 60:
    if int(axis.current_state) == 1:
        break
    time.sleep(0.2)
```

### Start both wheels in opposite directions at low speed
```python
import time
import odrive

odrv = odrive.find_any(timeout=12)
for axis in (odrv.axis0, odrv.axis1):
    axis.clear_errors()
    axis.controller.config.control_mode = 2
    axis.requested_state = 8

time.sleep(0.5)
odrv.axis0.controller.input_vel = 0.10
odrv.axis1.controller.input_vel = -0.10
```

### Stop both wheels
```python
odrv.axis0.controller.input_vel = 0
odrv.axis1.controller.input_vel = 0
```

## Final conclusion
The hoverboard wheel calibration with ODrive is now working for both wheels.

Final outcome:
- `M0` calibrated successfully
- `M1` calibrated successfully after hall pin rewiring
- both axes are healthy
- both axes can run at low speed
- both axes can run in opposite directions
- live encoder rotations can be read and printed during motion
