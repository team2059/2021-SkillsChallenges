package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ConveyorConstants;
import hhCore.config.RobotState;
import hhCore.subsystems.HHSubsystemBase;
import hhCore.utils.control.HHTalonSRX;

public class BallElevator extends HHSubsystemBase {

    VictorSPX ballElevator = new VictorSPX(ConveyorConstants.BallElevatorTalonPort);

    DigitalInput chamberPhotoElectricSensor = new DigitalInput(1);

    public BallElevator() {
        super("Conveyor");
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Chamber", isBallInChamber());
    }

    @Override
    public void update(RobotState robotState) {

    }

    public void autoLoader(double speed) {
        if (!isBallInChamber()){
            setBallElevatorMotor(speed);
        } else {
            setBallElevatorMotor(0);
        }
    }

    public boolean isBallInChamber() {
        return chamberPhotoElectricSensor.get();
    }

    public void setBallElevatorMotor(double speed) {
        ballElevator.set(ControlMode.PercentOutput, speed);
    }

}
