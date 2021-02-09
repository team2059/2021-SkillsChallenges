package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import hhCore.config.RobotState;
import hhCore.subsystems.HHSubsystemBase;
import hhCore.utils.control.HHTalonSRX;
import hhCore.utils.vision.LimeLight;

public class Turret extends HHSubsystemBase {

    HHTalonSRX turretRotator = new HHTalonSRX(TurretConstants.TurretRotatorTalonPort);

    DigitalInput leftHFX = new DigitalInput(TurretConstants.LeftLimitHallEffectPort);
    DigitalInput rightHFX = new DigitalInput(TurretConstants.RightLimitHallEffectPort);

    static LimeLight turretLimelight = new LimeLight();


    public Turret() {
        super("Turret");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Location", turretRotator.getSelectedSensorPosition());
        SmartDashboard.putNumber("Limelight X Offset", turretLimelight.getHorizontalOffset());
        SmartDashboard.putBoolean("Left Limit", getLeftHFX());
        SmartDashboard.putBoolean("Right Limti", getRightHFX());
        SmartDashboard.putNumber("Est. Goal Dist.", getDistanceToTarget());
        updateTurretLocation();
    }

    @Override
    public void update(RobotState robotState) {

    }

    public double getTurretEncoderLocation() {
        return turretRotator.getSelectedSensorPosition();
    }

    public void updateTurretLocation() {
        if (getLeftHFX()) {
            turretRotator.setSelectedSensorPosition(0);
        }

        if (getRightHFX()) {
            turretRotator.setSelectedSensorPosition(35165);
        }
    }

    public LimeLight getTurretLimelight() {
        return turretLimelight;
    }

    public boolean getLeftHFX() {
        return !leftHFX.get();
    }

    public boolean getRightHFX() {
        return !rightHFX.get();
    }

    public void rotateTurret() {
        if(Math.abs(RobotContainer.driveJS.getRawAxis(2)) > .2) { //Dummy Deadzone
            setTurretRotatorMotor(RobotContainer.driveJS.getRawAxis(2));
        } else {
            setTurretRotatorMotor(0);
        }
    }

    public double getHorizontalOffset() {
        return turretLimelight.getHorizontalOffset();
    }

//    public double hoodPositionToTarget() {
//
//    }

    public void setTurretRotatorMotor(double speed) {
//        if (getTurretEncoderLocation() >= TurretConstants.RightLimit && speed > 0) {
//            turretRotator.set(0);
//        } else if (getTurretEncoderLocation() <= TurretConstants.LeftLimit && speed < 0) {
//            turretRotator.set(0);
//        } else {
//            turretRotator.set(ControlMode.PercentOutput, speed);
//        }

        if (getLeftHFX() && speed < 0) {
            turretRotator.set(0);
        } else if (getRightHFX() && speed > 0) {
            turretRotator.set(0);
        } else {
            turretRotator.set(ControlMode.PercentOutput, speed);
        }

    }

    public double getDistanceToTarget() {
        return (FieldConstants.HighGoalHeight - FieldConstants.LimeLightHeight) / Math.tan(turretLimelight.getVerticalOffset() + FieldConstants.LimelightAngle);
    }
}
