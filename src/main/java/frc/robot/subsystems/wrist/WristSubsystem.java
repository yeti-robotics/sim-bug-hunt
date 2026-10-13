package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.sim.PhysicsSim;
import frc.robot.util.sim.SimulatableMechanism;

public class WristSubsystem extends SubsystemBase implements SimulatableMechanism {
    private final TalonFX wristMotor = new TalonFX(WristConfig.WRIST_KRAKEN_ID);
    private final CANcoder wristEncoder = new CANcoder(WristConfig.WRIST_CANCODER_ID);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    public WristSubsystem() {
        wristMotor.getConfigurator().apply(WristConfig.wristMotorConfigs);
        wristEncoder.getConfigurator().apply(WristConfig.wristEncoderConfigs);

        PhysicsSim.getInstance().addTalonFX(wristMotor, wristEncoder);
    }

    /** Runs at a fixed duty cycle (-1 to 1), stopping when the command ends. */
    public Command runDutyCycle(double output) {
        return runEnd(
                () -> wristMotor.setControl(dutyCycleRequest.withOutput(MathUtil.clamp(output, -1, 1))),
                () -> wristMotor.setControl(dutyCycleRequest.withOutput(0)));
    }

    @Override
    public Angle getCurrentPosition() {
        return wristMotor.getPosition().getValue();
    }

    @Override
    public Angle getTargetPosition() {
        return Units.Rotations.of(wristMotor.getClosedLoopReference().getValue());
    }
}
