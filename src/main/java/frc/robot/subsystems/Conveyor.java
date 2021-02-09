package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ConveyorConstants;
import hhCore.config.RobotState;
import hhCore.subsystems.HHSubsystemBase;
import hhCore.utils.control.HHTalonSRX;

public class Conveyor extends HHSubsystemBase {
    WPI_VictorSPX collectorMotor = new WPI_VictorSPX(ConveyorConstants.CollectorMotor);
    DoubleSolenoid collectorSolenoid = new DoubleSolenoid(0, 1);
    DoubleSolenoid collectorAgitator = new DoubleSolenoid(2, 3);


    public Conveyor() {
        super("Conveyor");
    }

    @Override
    public void update(RobotState robotState) {

    }

    @Override
    public void periodic() {
    }

    public void setCollectorMotor(double speed) {
        collectorMotor.set(speed);
    }

    public void setCollectorSolenoid(DoubleSolenoid.Value state) {
        collectorSolenoid.set(state);
    }

    public void toggleCollector() {
        if (collectorSolenoid.get() == DoubleSolenoid.Value.kForward)
            setCollectorSolenoid(DoubleSolenoid.Value.kReverse);
        else
            setCollectorSolenoid(DoubleSolenoid.Value.kForward);
    }

    public void setCollectorAgitator(DoubleSolenoid.Value state) {
        collectorAgitator.set(state);
    }

    public void toggleAgitator() {
        if (collectorAgitator.get() == DoubleSolenoid.Value.kForward)
            setCollectorAgitator(DoubleSolenoid.Value.kReverse);
        else
            setCollectorAgitator(DoubleSolenoid.Value.kForward);
    }
}
