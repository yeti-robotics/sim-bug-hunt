package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.sim.PhysicsSim;
import frc.robot.util.sim.SimulatableMechanism;

public class ElevatorSubsystem extends SubsystemBase implements SimulatableMechanism {
    private final TalonFX primaryElevatorMotor = new TalonFX(ElevatorConfig.primaryElevatorMotorID);
    private final TalonFX secondaryElevatorMotor = new TalonFX(ElevatorConfig.secondaryElevatorMotorID);
    private final DigitalInput magSwitch = new DigitalInput(ElevatorConfig.magSwitchID);
    private final NeutralOut neutralOut = new NeutralOut();
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    public ElevatorSubsystem() {
        primaryElevatorMotor.getConfigurator().apply(ElevatorConfig.primaryTalonFXConfigs);
        secondaryElevatorMotor.getConfigurator().apply(ElevatorConfig.secondaryTalonFXConfigs);
        secondaryElevatorMotor.setControl(
                new Follower(ElevatorConfig.primaryElevatorMotorID, MotorAlignmentValue.Opposed));
        new Trigger(this::getMagSwitch).debounce(2).onTrue(zeroPosition());

        primaryElevatorMotor.setPosition(0);
        secondaryElevatorMotor.setPosition(0);

        primaryElevatorMotor.setControl(neutralOut);

        PhysicsSim.getInstance().addTalonFX(primaryElevatorMotor);
        PhysicsSim.getInstance().addTalonFX(secondaryElevatorMotor);
    }

    /** Runs both elevator motors at a fixed duty cycle, stopping when the command ends. */
    public Command runDutyCycle(double output) {
        return runEnd(
                () -> primaryElevatorMotor.setControl(dutyCycleRequest.withOutput(MathUtil.clamp(output, -1, 1))),
                () -> primaryElevatorMotor.setControl(dutyCycleRequest.withOutput(0)));
    }

    @Override
    public Angle getCurrentPosition() {
        return primaryElevatorMotor.getPosition().getValue();
    }

    @Override
    public Angle getTargetPosition() {
        return Units.Rotations.of(primaryElevatorMotor.getClosedLoopReference().getValue());
    }

    private Command zeroPosition() {
        // Resetting the sensor must not interrupt a held manual movement command.
        return Commands.runOnce(() -> primaryElevatorMotor.setPosition(0));
    }

    public boolean getMagSwitch() {
        return !magSwitch.get();
    }
}
