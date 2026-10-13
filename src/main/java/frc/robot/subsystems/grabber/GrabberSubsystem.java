package frc.robot.subsystems.grabber;

import static frc.robot.constants.Constants.RIO_BUS;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.sim.PhysicsSim;

@Logged
public class GrabberSubsystem extends SubsystemBase {
    private final TalonFX claw = new TalonFX(GrabberConfig.CLAW_ID, RIO_BUS);
    private final DutyCycleOut dutyCycleReq = new DutyCycleOut(0);
    // Canandcolor has no simulated sensor here; only connect to it on hardware.
    private final Canandcolor clawSwitch =
            RobotBase.isReal() ? new Canandcolor(GrabberConfig.GRABBER_CANANDCOLOR) : null;

    // Your job: initialize and create triggers for the grabber's Canandcolor!

    public GrabberSubsystem() {
        claw.getConfigurator().apply(GrabberConfig.coralMotorConfig);
        claw.setControl(dutyCycleReq);
        PhysicsSim.getInstance().addTalonFX(claw);
    }

    /** Runs at a fixed duty cycle (-1 to 1), stopping when the command ends. */
    public Command runDutyCycle(double output) {
        return runEnd(
                () -> claw.setControl(dutyCycleReq.withOutput(MathUtil.clamp(output, -1, 1))),
                () -> claw.setControl(dutyCycleReq.withOutput(0)));
    }

    public Command intake() {
        return runDutyCycle(GrabberConfig.INTAKE);
    }

    public Command outtake() {
        return runDutyCycle(GrabberConfig.OUTSPIT);
    }

    // Define the trigger conditions!

    public boolean hasCoral() {
        return false;
    }

    public boolean doesNotHaveCoral() {
        return false;
    }

    public boolean hasAlgae() {
        return false;
    }

    public boolean doesNotHaveAlgae() {
        return false;
    }
}
