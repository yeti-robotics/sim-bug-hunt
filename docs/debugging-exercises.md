# Robot debugging exercises

Your job is to repair six small mistakes in an otherwise working simulated robot.
The project should compile before you fix anything. The README lists the intended
controls and motor outputs.

## Get started

1. Run `./gradlew simulateJava`.
2. Select **Teleoperated** and enable the robot in the simulation Driver Station.
3. Keep the simulation window focused. Use W/S, A/D, J/K, and the number row 1–8.
4. Connect AdvantageScope to the simulation's NetworkTables server. Use `/Pose`
   and `ComponentPoses/Real` for the robot model, and
   `SmartDashboard/Mechanisms/CoralManipulator` for the elevator/arm diagram.

Use the current-position topic (`ComponentPoses/Real`) for these exercises. The
target-position topic is for closed-loop targets; these controls run motors manually.
The grabber roller is not animated in the model, so use its test failure to investigate
exercise 5.

## First round: controls

| Exercise | Try this | Expected behavior |
| --- | --- | --- |
| 1. Driving | Press W/S, then A/D, with the robot at its initial heading. | W/S move forward/backward; A/D strafe. |
| 2. Elevator | Hold 1 briefly, release, then hold 2 briefly. | The two buttons move the elevator in opposite directions. |
| 3. Wrist controls | Hold 5 briefly, release, then hold 6 briefly. | Both controls work and request opposite directions. |

## Second round: commands and displayed behavior

| Exercise | Try this | Expected behavior |
| --- | --- | --- |
| 4. Arm release | Hold 3 briefly and release it. Compare with releasing an elevator control. | The motor output becomes zero when the button is released. A brief coast is different from remaining powered. |
| 5. Grabber direction | Run the tests and inspect the grabber movement failure. | Intake (7) requests +25%; outtake (8) requests -25%. |
| 6. Wrist display | After repairing the wrist controls, make a small wrist movement and observe the model. | The model's wrist angle agrees with the measured angle; it should not turn many times for a small movement. |

## Work like a debugger

For each exercise:

1. Write down what you expected and what actually happened.
2. Decide what evidence would help: joystick input, requested motor output, sensor
   position, or the displayed pose.
3. Trace that evidence through the code. Compare a similar working control or subsystem.
4. Make one small change and repeat the same experiment.
5. Run `./gradlew test` and check whether the relevant failure disappeared.
6. Explain why your change fixes the cause.

Run `./gradlew compileJava` for a quick compilation check. The full test report is
`build/reports/tests/test/index.html`. Multiple failing tests can share one cause;
there are six faults, not necessarily six failures. Repair the robot code rather
than weakening test assertions.

Restart the simulation after code changes and between experiments if a motor keeps
running. Use Disable to stop powered movement while investigating.

Completion means the intended controls work, the model agrees with the sensors,
and all tests pass.
