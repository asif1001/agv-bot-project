
Goal:
Connect to my ODrive over USB and calibrate both hoverboard wheels:
- `axis0 / M0`
- `axis1 / M1`

System and hardware:
- Raspberry Pi 4
- ODrive v3.6
- Firmware: 0.5.1
- Hoverboard wheels with hall sensors
- Encoder CPR: 90
- Pole pairs: 15
- Wheel diameter: 6.5 inches
- Wheel circumference: about 0.518 meters

Important:
- Use only safe, low-risk steps
- The wheels may spin during calibration, so assume they must be free to rotate
- Diagnose first if there are errors
- Do not force wheel motion if there are active encoder or hall faults
- Use the exact ODrive configuration below

Python environment:
- If needed, use a Python virtual environment with the `odrive` package installed
- If there is already a working ODrive venv, use it

Required ODrive configuration for BOTH axes:
```python
axis.motor.config.pole_pairs = 15
axis.motor.config.motor_type = 0
axis.motor.config.calibration_current = 8
axis.motor.config.resistance_calib_max_voltage = 4
axis.encoder.config.mode = 1
axis.encoder.config.cpr = 90
axis.encoder.config.bandwidth = 100
```

Calibration workflow:
1. Detect the ODrive on USB
2. Connect to it with Python `odrive`
3. Inspect current errors on both axes
4. For `axis0`:
   - clear errors
   - apply the hoverboard config
   - request full calibration sequence
   - wait until the axis returns to idle
   - verify no errors remain
5. For `axis1`:
   - clear errors
   - apply the hoverboard config
   - request full calibration sequence
   - wait until the axis returns to idle
   - verify no errors remain
6. After both calibrations, read back the final status for both axes

Success criteria for each axis:
- `axis.error == 0`
- `axis.motor.error == 0`
- `axis.encoder.error == 0`
- `axis.controller.error == 0`
- `axis.motor.is_calibrated == True`
- `axis.encoder.is_ready == True`

Expected calibration state progression:
- `4`
- `7`
- `1`

If an axis fails with hall-related errors such as:
- `AXIS_ERROR_ENCODER_FAILED`
- `ENCODER_ERROR_ILLEGAL_HALL_STATE`

Then:
- do not force motion
- report that the hall wiring is wrong or unstable
- tell me to check:
  - 5V
  - GND
  - Hall A
  - Hall B
  - Hall C
  - connector seating
  - whether motor phases and hall wires belong to the same wheel

If ODrive connection times out:
- check USB detection
- check `/dev/ttyACM0`
- check for stale ODrive processes
- retry one axis at a time instead of doing too much in one long session

Commands/checks to run first:
```bash
lsusb | grep -i odrive || true
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true
pgrep -af "odrive|odrivetool|odrive_dashboard" || true
```

Use Python with `odrive` to inspect:
- serial number
- bus voltage
- `axis0` errors
- `axis1` errors
- calibration state
- encoder readiness

Example Python script pattern:
```python
import time
import odrive

odrv = odrive.find_any(timeout=12)

def calibrate(axis, name):
    axis.clear_errors()

    axis.motor.config.pole_pairs = 15
    axis.motor.config.motor_type = 0
    axis.motor.config.calibration_current = 8
    axis.motor.config.resistance_calib_max_voltage = 4

    axis.encoder.config.mode = 1
    axis.encoder.config.cpr = 90
    axis.encoder.config.bandwidth = 100

    print(name, "after clear:")
    print(" axis_error =", int(axis.error))
    print(" motor_error =", int(axis.motor.error))
    print(" encoder_error =", int(axis.encoder.error))

    axis.requested_state = 3

    start = time.time()
    last_state = None
    while time.time() - start < 60:
        state = int(axis.current_state)
        if state != last_state:
            print(name, "state =", state)
            last_state = state
        if state == 1:
            break
        time.sleep(0.2)

    print(name, "final:")
    print(" axis_error =", int(axis.error))
    print(" motor_error =", int(axis.motor.error))
    print(" encoder_error =", int(axis.encoder.error))
    print(" controller_error =", int(axis.controller.error))
    print(" motor_is_calibrated =", bool(getattr(axis.motor, "is_calibrated", False)))
    print(" encoder_is_ready =", bool(getattr(axis.encoder, "is_ready", False)))

calibrate(odrv.axis0, "axis0")
calibrate(odrv.axis1, "axis1")
```

After calibration, print final status for both axes:
- current state
- requested state
- axis error
- motor error
- encoder error
- controller error
- `motor.is_calibrated`
- `encoder.is_ready`
- encoder mode
- encoder CPR
- position estimate
- velocity estimate

Report back:
1. whether ODrive was detected
2. whether `axis0 / M0` calibrated successfully
3. whether `axis1 / M1` calibrated successfully
4. exact error codes if anything failed
5. final ready state for both wheels

Do not save config unless I explicitly ask.
Do not run high-speed wheel motion.
If calibration succeeds, stop at verification and wait for my next instruction.
```

# Short Version

```text
Calibrate both hoverboard wheels on my Raspberry Pi 4 using ODrive.

Hardware:
- ODrive v3.6
- Firmware 0.5.1
- Hall sensors
- axis0 = M0
- axis1 = M1
- pole_pairs = 15
- encoder_cpr = 90
- calibration_current = 8
- resistance_calib_max_voltage = 4
- encoder_mode = 1
- encoder_bandwidth = 100

For both axes:
- clear errors
- apply config
- run full calibration sequence
- wait for idle
- verify:f
  - axis error = 0
  - motor error = 0
  - encoder error = 0
  - controller error = 0
  - motor_is_calibrated = True
  - encoder_is_ready = True
