# Mentor answer key — spoilers

The exercise contains the six approved faults. Optional integer-division and missing
simulation-registration exercises were not introduced.

Before inserting the faults, the control documentation and tests were corrected to
use buttons 5/6 for the wrist and 7/8 for the grabber, with nine simulated buttons
available. All 15 tests passed with these corrections and the new drive/pose checks.

## Faults and repairs

### 1. Driving axes swapped

**Location:** `src/main/java/frc/robot/RobotContainer.java`, `configureBindings()`.

**Symptom:** At the initial heading, W/S strafe and A/D move forward/backward.

**Repair:** Use `-joystick.getY()` for `withVelocityX(...)` and
`-joystick.getX()` for `withVelocityY(...)`. Keep the existing speed scaling and
rotation binding.

**Check:** `driveAxesMatchForwardAndStrafeControls` passes. Holding one translation
axis should request velocity along only its intended direction.

**Hint ladder:** Check the joystick axes → compare forward and strafe bindings →
follow each getter into the requested velocity component.

### 2. Elevator down has the wrong sign

**Location:** `src/main/java/frc/robot/RobotContainer.java`, button 2.

**Symptom:** Buttons 1 and 2 both request positive elevator output.

**Repair:** Pass `-Constants.ELEVATOR_MANUAL_DUTY_CYCLE` for button 2.

**Check:** The elevator case of `controlsMoveAndStop` passes, including opposite
physical movement and stopping on release.

**Hint ladder:** Compare the two requested outputs → compare the arguments in the
bindings → determine which direction needs a negative value.

### 3. Wrist reverse bound to button 9

**Location:** `src/main/java/frc/robot/RobotContainer.java`, negative wrist binding.

**Symptom:** Key 6 does nothing; key 9 unexpectedly requests reverse wrist output.
The keyboard setup already exposes key 9, so this is a functioning wrong binding.

**Repair:** Change `joystick.button(9)` to `joystick.button(6)`.

**Check:** The wrist case of `controlsMoveAndStop` passes. A separate display error
remains until fault 6 is repaired.

**Hint ladder:** Check the intended control table → observe joystick button 6 →
find the wrist reverse binding.

### 4. Arm command does not stop its motor

**Location:** `src/main/java/frc/robot/subsystems/arm/ArmSubsystem.java`, `runDutyCycle()`.

**Symptom:** Releasing the arm button cancels the command, but the motor retains its
last nonzero duty-cycle request. It also retains the request when interrupted by
another command. Disabling removes powered output, but does not repair the missing
command cleanup.

**Repair:** Restore the command's end action:

```java
return runEnd(
        () -> armKraken.setControl(dutyCycleRequest.withOutput(MathUtil.clamp(output, -1, 1))),
        () -> armKraken.setControl(dutyCycleRequest.withOutput(0)));
```

**Check:** The arm cases of `controlsMoveAndStop`, `dutyCycleIsClamped`, and
`disableAndInterruptionStopOutput` pass. Three failures arise from this one fault.

**Hint ladder:** Is the command still scheduled? → does cancelling a command send a
new motor request? → compare the arm command with the elevator command.

### 5. Grabber outtake has the wrong sign

**Location:** `src/main/java/frc/robot/subsystems/grabber/GrabberConfig.java`, `OUTSPIT`.

**Symptom:** Both intake and outtake request +25% duty cycle. The grabber roller is
not animated in the model; use the grabber case of `controlsMoveAndStop` to inspect
the requested output and simulated movement.

**Repair:** Set `OUTSPIT = -0.25`.

**Check:** The grabber movement test passes, including reversed movement.

**Hint ladder:** The buttons call different methods; what do those methods return? →
trace the values passed to `runDutyCycle()` → compare intake/outtake configuration.

### 6. Wrist display passes degrees into a radians constructor

**Location:** `src/main/java/frc/robot/util/sim/Mechanisms.java`, `publishComponentPoses()`.

**Symptom:** The displayed wrist rotation changes much faster than the measured
angle. Both current and target pose publishers use this conversion, though the
manual-control exercise should be observed with `ComponentPoses/Real`.

**Repair:** Change `wristPos.in(Degrees)` to `wristPos.in(Radians)`.
`Rotation3d` takes radians. A measured 30 degrees should display as pi/6 radians;
the faulty code passes 30 radians instead. Euler angles wrap, so the test compares
the wrist's rotation relative to the arm rather than expecting an unwrapped angle.

**Check:** `displayedWristRotationMatchesMeasuredAngle` passes.

**Hint ladder:** Is the sensor value reasonable? → compare the arm and wrist
conversions → check the units accepted by `Rotation3d`.

## Teaching and verification

Start with faults 1–3, then 4–6. Have students repair fault 3 before investigating
the wrist display. Require an observation, a hypothesis, a small change, and an
explanation rather than a list of edited lines.

The tests reset motor requests between cases so the arm's stale output does not
contaminate unrelated checks. Assertions describe correct behavior throughout;
none were changed to accept faulty behavior.

With all six faults present, expect 15 tests with eight failures: the drive check,
four movement/release cases, two additional arm cases, and the wrist display check.
After all repairs, all 15 should pass.

Use `./gradlew compileJava spotlessCheck` to check compilation and formatting.
Use `./gradlew test` to evaluate behavior; failure is expected for the exercise
checkout. Read `build/reports/tests/test/index.html` for individual assertions.
The existing real-robot execution guard in `Robot.robotInit()` remains in place.
