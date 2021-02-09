package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.RobotContainer;
import hhCore.config.RobotState;
import hhCore.subsystems.HHSubsystemBase;

public class Climber extends HHSubsystemBase {
    WPI_TalonSRX climberMotor = new WPI_TalonSRX(ClimberConstants.ArmLifterMotorPort);
    WPI_VictorSPX trunionMotor = new WPI_VictorSPX(ClimberConstants.TrunionMotorPort);
    WPI_VictorSPX winchMotor = new WPI_VictorSPX(ClimberConstants.WinchMotorPort);

    public Climber() {
        super("Climber");
        climberMotor.configFactoryDefault();
        climberMotor.setInverted(true);
//        climberMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, ClimberConstants.LoopIdx, ClimberConstants.CtreTimeoutMs);
        climberMotor.setSelectedSensorPosition(0);

//        climberMotor.configNominalOutputForward(0, ClimberConstants.CtreTimeoutMs);
//        climberMotor.configNominalOutputReverse(0, ClimberConstants.CtreTimeoutMs);
//        climberMotor.configPeakOutputForward(1, ClimberConstants.CtreTimeoutMs);
//        climberMotor.configPeakOutputReverse(-1, ClimberConstants.CtreTimeoutMs);
//
//        climberMotor.selectProfileSlot(ClimberConstants.MotorSlotIdx, Constants.ShooterConstants.LoopIdx);
//        climberMotor.config_kF(ClimberConstants.MotorSlotIdx, ClimberConstants.ArmKf, ClimberConstants.CtreTimeoutMs);
//        climberMotor.config_kP(ClimberConstants.MotorSlotIdx, ClimberConstants.ArmKp, ClimberConstants.CtreTimeoutMs);
//        climberMotor.config_kI(ClimberConstants.MotorSlotIdx, ClimberConstants.ArmkI, ClimberConstants.CtreTimeoutMs);
//        climberMotor.config_kD(ClimberConstants.MotorSlotIdx, ClimberConstants.ArmkD, ClimberConstants.CtreTimeoutMs);

        /* Set acceleration and vcruise velocity - see documentation */
//        climberMotor.configMotionCruiseVelocity(3500, ClimberConstants.CtreTimeoutMs);
//        climberMotor.configMotionAcceleration(4000, ClimberConstants.CtreTimeoutMs);
    }

    @Override
    public void update(RobotState robotState) {

    }

    @Override
    public void periodic() {
//        SmartDashboard.putNumber("Output Percent", climberMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Climber Encoder Positon", climberMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Climber Voltage", climberMotor.getBusVoltage());

    }

    public double getArmLocation() {
        return climberMotor.getSelectedSensorPosition();
    }

    public void holdArmPosition() {
        System.out.println("Setting Arm to " + getArmLocation());
        climberMotor.set(ControlMode.Position, getArmLocation());
    }

    public void setClimberMotor(double speed) {
        climberMotor.set(speed);
    }

    public void setArmPosition(double location) {
        System.out.println("Starting Motion Magic to " + location);
//        climberMotor.set(ControlMode.MotionMagic, location);
    }

    public void setWinchMotor(double speed) {
        winchMotor.set(speed);
    }

    public void setTrunionMotor(double speed) {
        trunionMotor.set(speed);
    }
}
