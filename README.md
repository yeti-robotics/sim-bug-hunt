# Robot simulation sandbox

Practice command-based FRC programming with simulated swerve drive, elevator, arm,
wrist, and grabber motors.

This checkout contains six intentional debugging faults. Start with the
[student exercises](docs/debugging-exercises.md). The controls and behavior below
describe how the robot should work after repairs.

Run `./gradlew simulateJava`, select **Teleoperated**, and enable the robot in the
simulation Driver Station. The checked-in `simgui-ds.json` maps **Keyboard 0** to
joystick port **0**. Keep the simulation window focused to use its keyboard controls.

| Mechanism | Joystick input | Keyboard control | Duty cycle |
| --- | --- | --- | --- |
| Drive forward / backward | Y axis | W / S | Existing open-loop swerve control |
| Strafe left / right | X axis | A / D | Existing open-loop swerve control |
| Rotate | Twist axis | J / K | Existing open-loop swerve control |
| Elevator up / down | Buttons 1 / 2 | 1 / 2 | +10% / -10% |
| Arm positive / negative | Buttons 3 / 4 | 3 / 4 | +10% / -10% |
| Wrist positive / negative | Buttons 5 / 6 | 5 / 6 | +5% / -5% |
| Grabber intake / outtake | Buttons 7 / 8 | 7 / 8 | +25% / -25% |

Hold a control to move a mechanism. Correct behavior is to set motor output to zero
when releasing it, disabling the robot, or interrupting its command. A physical joystick on port 0 uses the
same button bindings. Use the number row on the keyboard; a number pad is not needed.

Each motor subsystem exposes `runDutyCycle(double output)`, where `-1.0` is full
reverse, `0.0` is stopped, and `1.0` is full forward. The command requires its
subsystem and should repeatedly apply the output and stop the motor when it ends:

```java
joystick.button(1).whileTrue(elevator.runDutyCycle(0.1));
```

The elevator's second motor follows its primary motor. Manual speeds are in
`Constants.java`; grabber intake and outtake speeds are in `GrabberConfig.java`.
In AdvantageScope, connect to the simulation's NetworkTables server and use
`/Pose`, `ComponentPoses/Real`, and `SmartDashboard/Mechanisms/CoralManipulator`
to view the drivetrain and moving mechanisms. The grabber roller spins internally;
the component poses show the wrist and grabber assembly's orientation.

Run `./gradlew test` to check keyboard-equivalent joystick bindings, simulated motor
movement in both directions, stopping on release, disable, and interruption, and
the displayed wrist angle. Tests intentionally fail until the faults are repaired.
Open `build/reports/tests/test/index.html` to read individual failures.

Mentors: the [answer key](docs/mentor-answer-key.md) contains spoilers and suggested
teaching order. Keep it out of the student handout.
