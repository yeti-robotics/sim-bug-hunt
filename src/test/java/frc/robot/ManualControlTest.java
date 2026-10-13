package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.sim.Mechanisms;
import frc.robot.util.sim.PhysicsSim;
import java.lang.reflect.Field;
import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

class ManualControlTest {
    private static final CommandScheduler scheduler = CommandScheduler.getInstance();
    private static final JoystickSim joystick = new JoystickSim(0);
    private static RobotContainer robot;

    @BeforeAll
    static void initializeSimulation() {
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(false);
        joystick.setAxisCount(3);
        joystick.setButtonCount(9);
        joystick.setPOVCount(1);
        DriverStationSim.notifyNewData();
        robot = new RobotContainer();
        // The active-low elevator switch is unpressed for these manual movement tests.
        new DIOSim(6).setValue(true);
    }

    @BeforeEach
    void resetControls() throws Exception {
        scheduler.cancelAll();
        // A failed stop assertion must not leave output running in the next test.
        for (Arguments mechanism : mechanisms().toList()) {
            Object[] names = mechanism.get();
            TalonFX motor = (TalonFX) field(field(robot, (String) names[0]), (String) names[1]);
            motor.setControl(new DutyCycleOut(0));
        }
        for (int button = 1; button <= 9; button++) {
            joystick.setRawButton(button, false);
        }
        for (int axis = 0; axis < 3; axis++) {
            joystick.setRawAxis(axis, 0);
        }
        joystick.setPOV(-1);
        new DIOSim(6).setValue(true);
        DriverStationSim.setEnabled(true);
        runCycles(2);
    }

    static Stream<Arguments> mechanisms() {
        return Stream.of(
                Arguments.of("elevator", "primaryElevatorMotor", 1, 2),
                Arguments.of("arm", "armKraken", 3, 4),
                Arguments.of("wrist", "wristMotor", 5, 6),
                Arguments.of("grabber", "claw", 7, 8));
    }

    @Test
    void driveAxesMatchForwardAndStrafeControls() throws Exception {
        SwerveRequest.FieldCentric request = (SwerveRequest.FieldCentric) field(robot, "drive");
        joystick.setRawAxis(1, -0.5);
        runCycles(3);
        assertTrue(request.VelocityX > 0, "Forward input must request forward velocity");
        assertEquals(0, request.VelocityY, 1e-9, "Forward input must not request strafe velocity");

        joystick.setRawAxis(1, 0);
        joystick.setRawAxis(0, -0.5);
        runCycles(3);
        assertEquals(0, request.VelocityX, 1e-9, "Strafe input must not request forward velocity");
        assertTrue(request.VelocityY > 0, "Strafe input must request sideways velocity");
    }

    @Test
    void displayedWristRotationMatchesMeasuredAngle() {
        try (var poses = NetworkTableInstance.getDefault()
                .getStructArrayTopic("ComponentPoses/Real", Pose3d.struct)
                .subscribe(new Pose3d[0])) {
            Mechanisms mechanisms = new Mechanisms();
            mechanisms.publishComponentPoses(Units.Rotations.of(0), Units.Degrees.of(0), Units.Degrees.of(30), true);
            Pose3d[] components = poses.get();
            assertEquals(4, components.length);
            double wristPitch =
                    components[3].relativeTo(components[2]).getRotation().getY();
            assertEquals(Math.PI / 6, wristPitch, 1e-9, "A measured 30-degree wrist angle must display as 30 degrees");
        }
    }

    @ParameterizedTest(name = "{0} moves in both directions and stops on release")
    @MethodSource("mechanisms")
    void controlsMoveAndStop(String subsystemName, String motorName, int positive, int negative) throws Exception {
        Object subsystem = field(robot, subsystemName);
        TalonFX motor = (TalonFX) field(subsystem, motorName);
        assertInstanceOf(Subsystem.class, subsystem, "The mechanism must participate in command scheduling");

        double positiveDelta = 0;
        for (int direction : new int[] {positive, negative}) {
            joystick.setRawButton(direction, true);
            runCycles(15);
            DutyCycleOut request = assertInstanceOf(
                    DutyCycleOut.class, motor.getAppliedControl(), "Holding a control must command duty-cycle output");
            assertTrue(
                    direction == positive ? request.Output > 0 : request.Output < 0,
                    "Button " + direction + " must request " + (direction == positive ? "positive" : "negative")
                            + " output");
            assertTrue(Math.abs(motor.getSimState().getMotorVoltage()) > 0.1, "The simulated motor must be powered");
            if (subsystemName.equals("elevator")) {
                TalonFX follower = (TalonFX) field(subsystem, "secondaryElevatorMotor");
                assertEquals(
                        -motor.getSimState().getMotorVoltage(),
                        follower.getSimState().getMotorVoltage(),
                        0.1,
                        "The opposed elevator follower must be powered with the primary motor");
            }
            double initialPosition = motor.getPosition().refresh().getValueAsDouble();
            runCycles(15);
            double positionDelta = motor.getPosition().refresh().getValueAsDouble() - initialPosition;
            assertTrue(Math.abs(positionDelta) > 0.001, "The simulated position must change while powered");
            if (direction == positive) {
                positiveDelta = positionDelta;
            } else {
                assertTrue(positiveDelta * positionDelta < 0, "The second input must reverse physical movement");
            }

            joystick.setRawButton(direction, false);
            runCycles(3);
            assertStopped(motor);
        }
    }

    @ParameterizedTest(name = "{0} limits duty cycle to the motor's valid range")
    @MethodSource("mechanisms")
    void dutyCycleIsClamped(String subsystemName, String motorName, int positive, int negative) throws Exception {
        Object subsystem = field(robot, subsystemName);
        TalonFX motor = (TalonFX) field(subsystem, motorName);
        for (double output : new double[] {2, -2}) {
            Command command = (Command)
                    subsystem.getClass().getMethod("runDutyCycle", double.class).invoke(subsystem, output);
            scheduler.schedule(command);
            runCycles(3);
            assertEquals(
                    output > 0 ? 1.0 : -1.0,
                    assertInstanceOf(DutyCycleOut.class, motor.getAppliedControl()).Output,
                    1e-9);
            scheduler.cancel(command);
            runCycles(3);
            assertStopped(motor);
        }
    }

    @Test
    void elevatorSensorZeroingDoesNotInterruptHeldMovement() throws Exception {
        Object elevator = field(robot, "elevator");
        TalonFX motor = (TalonFX) field(elevator, "primaryElevatorMotor");
        motor.setPosition(1);
        joystick.setRawButton(1, true);
        runCycles(3);
        Command movement = scheduler.requiring(assertInstanceOf(Subsystem.class, elevator));
        assertNotNull(movement);
        new DIOSim(6).setValue(false);
        double previousPosition = motor.getPosition().refresh().getValueAsDouble();
        boolean positionWasReset = false;
        for (int i = 0; i < 110; i++) {
            runCycles(1);
            double currentPosition = motor.getPosition().refresh().getValueAsDouble();
            positionWasReset |= currentPosition < previousPosition - 0.5;
            previousPosition = currentPosition;
        }
        assertTrue(positionWasReset, "The debounced switch must reset the elevator's sensor position");
        assertSame(movement, scheduler.requiring((Subsystem) elevator));
        assertTrue(assertInstanceOf(DutyCycleOut.class, motor.getAppliedControl()).Output > 0);
        new DIOSim(6).setValue(true);
        joystick.setRawButton(1, false);
        runCycles(3);
        assertStopped(motor);
    }

    @ParameterizedTest(name = "{0} stops on disable and command interruption")
    @MethodSource("mechanisms")
    void disableAndInterruptionStopOutput(String subsystemName, String motorName, int positive, int negative)
            throws Exception {
        Subsystem subsystem = assertInstanceOf(Subsystem.class, field(robot, subsystemName));
        TalonFX motor = (TalonFX) field(subsystem, motorName);
        joystick.setRawButton(positive, true);
        runCycles(3);
        assertTrue(assertInstanceOf(DutyCycleOut.class, motor.getAppliedControl()).Output > 0);

        Command replacement = subsystem.run(() -> {});
        scheduler.schedule(replacement);
        runCycles(3);
        assertStopped(motor);
        scheduler.cancel(replacement);

        joystick.setRawButton(positive, false);
        runCycles(2);
        joystick.setRawButton(positive, true);
        runCycles(3);
        DriverStationSim.setEnabled(false);
        runCycles(3);
        assertStopped(motor);
    }

    private static void assertStopped(TalonFX motor) {
        assertEquals(0.0, assertInstanceOf(DutyCycleOut.class, motor.getAppliedControl()).Output, 1e-9);
        assertEquals(0.0, motor.getSimState().getMotorVoltage(), 0.05);
    }

    private static void runCycles(int count) {
        for (int i = 0; i < count; i++) {
            DriverStationSim.notifyNewData();
            Unmanaged.feedEnable(100);
            scheduler.run();
            PhysicsSim.getInstance().run();
            robot.updateVisionSim();
            robot.updateMechanisms();
            Timer.delay(0.02);
        }
    }

    private static Object field(Object owner, String name) throws Exception {
        Field field = owner.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.get(owner);
    }
}
