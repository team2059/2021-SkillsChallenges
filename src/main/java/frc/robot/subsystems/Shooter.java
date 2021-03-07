package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
import hhCore.config.RobotState;
import hhCore.subsystems.HHSubsystemBase;

public class Shooter extends HHSubsystemBase {

    WPI_TalonSRX FlywheelShooter = new WPI_TalonSRX(ShooterConstants.ShooterFlywheelTalonPort);

    CANSparkMax HoodMotor = new CANSparkMax(ShooterConstants.HoodMotorSparkMaxPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANPIDController hoodPIDController;
        private CANEncoder hoodEncoder;
        private int smartMotionSlot = 1;

    AnalogInput hoodPot = new AnalogInput(ShooterConstants.HoodPotPort);

    SimpleMotorFeedforward flyWheelFeedforward = new SimpleMotorFeedforward(ShooterConstants.ksVolts, ShooterConstants.kvVoltSecondsPerMeter, ShooterConstants.kaVoltSecondsSquaredPerMeter);

    public static int targetVelocity = 0;
    public static double targetHoodPosition = 0;

    public Shooter() {
        super("Shooter");

        // Configure FlyWheel Talon SRX
        FlywheelShooter.configFactoryDefault();
        FlywheelShooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, ShooterConstants.LoopIdx, ShooterConstants.CtreTimeoutMs);
        FlywheelShooter.setSensorPhase(true);

        FlywheelShooter.configNominalOutputForward(0, ShooterConstants.CtreTimeoutMs);
        FlywheelShooter.configNominalOutputReverse(0, ShooterConstants.CtreTimeoutMs);
        FlywheelShooter.configPeakOutputForward(1, ShooterConstants.CtreTimeoutMs);
        FlywheelShooter.configPeakOutputReverse(-1, ShooterConstants.CtreTimeoutMs);

        FlywheelShooter.selectProfileSlot(ShooterConstants.MotorSlotIdx, ShooterConstants.LoopIdx);
        FlywheelShooter.config_kF(ShooterConstants.MotorSlotIdx, ShooterConstants.FlyWheelkF, ShooterConstants.CtreTimeoutMs);
        FlywheelShooter.config_kP(ShooterConstants.MotorSlotIdx, ShooterConstants.FlyWheelkP, ShooterConstants.CtreTimeoutMs);
        FlywheelShooter.config_kI(ShooterConstants.MotorSlotIdx, ShooterConstants.FlyWheelkI, ShooterConstants.CtreTimeoutMs);
        FlywheelShooter.config_kD(ShooterConstants.MotorSlotIdx, ShooterConstants.FlyWheelkD, ShooterConstants.CtreTimeoutMs);

        // Configure Hood Spark Max
        HoodMotor.setInverted(true);
        hoodPIDController = HoodMotor.getPIDController();
        hoodEncoder = HoodMotor.getEncoder();

        hoodPIDController.setP(ShooterConstants.HoodkP);
        hoodPIDController.setI(ShooterConstants.HoodkI);
        hoodPIDController.setD(ShooterConstants.HoodkD);
        hoodPIDController.setIZone(ShooterConstants.HoodkIz);
        hoodPIDController.setFF(ShooterConstants.HoodkFF);
        hoodPIDController.setOutputRange(ShooterConstants.HoodkMinOutput, ShooterConstants.HoodkMaxOutput);

        hoodPIDController.setSmartMotionMaxVelocity(ShooterConstants.HoodMaxVel, smartMotionSlot);
        hoodPIDController.setSmartMotionMinOutputVelocity(ShooterConstants.HoodMinVel, smartMotionSlot);
        hoodPIDController.setSmartMotionMaxAccel(ShooterConstants.HoodMaxAcc, smartMotionSlot);
        hoodPIDController.setSmartMotionAllowedClosedLoopError(ShooterConstants.HoodAllowedErr, smartMotionSlot);
    
        SmartDashboard.putNumber("Target Flywheel Velocity", targetVelocity);
        SmartDashboard.putNumber("Target Hood Position", targetHoodPosition);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("FlyWheel Velocity", FlywheelShooter.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Hood Position", hoodEncoder.getPosition());
        SmartDashboard.putNumber("Hood Pot Voltage", getHoodPotVoltage());

        Shooter.targetVelocity = (int) SmartDashboard.getNumber("Target Flywheel Velocity", 0);
        Shooter.targetHoodPosition = SmartDashboard.getNumber("Target Hood Position", 0);

//        SmartDashboard.putNumber("Hood Pot Voltage", getHoodPotVoltage());
        mapPotVoltageToNeoEncoder();
//        SmartDashboard.putNumber("Hood Output", HoodMotor.getAppliedOutput());

    }



    @Override
    public void update(RobotState robotState) {

    }

    public void mapPotVoltageToNeoEncoder() {
        if (Math.abs(getHoodPotVoltage() - .7885) < .1) {
            HoodMotor.getEncoder().setPosition(0);
        }

        if (Math.abs(getHoodPotVoltage() - 3.24) < .1) {
            HoodMotor.getEncoder().setPosition(18.142);
        }
    }

    public double getHoodPotVoltage() {
        return hoodPot.getVoltage();
    }

    public double getFlyWheelVelocity() {
        return FlywheelShooter.getSelectedSensorVelocity();
    }

    public void setHoodPosition(double position) {
        System.out.println("Setting Hood to " + position);
//        hoodPIDController.setReference(position, ControlType.kSmartMotion);
        HoodMotor.getPIDController().setReference(position, ControlType.kSmartMotion);
    }

    public double getHoodPosition() {
        return hoodEncoder.getPosition();
    }

    public void setHoodMotor(double speed) {
        HoodMotor.set(speed);
    }

    public double getWheelSpeed() {
        return FlywheelShooter.getSelectedSensorVelocity();
    }

    public void setWheelVelocity() {
        double velocity = Math.abs(RobotContainer.driveJS.getRawAxis(3) * 25000);
        System.out.println("Setting " + velocity + " ticks velocity");
        FlywheelShooter.set(ControlMode.Velocity, velocity);
    }

    // USING DYNAMIC FEEDFORWARD VALUE
    // public void setWheelVelocity(double speed) {
    //     double kF = flyWheelFeedforward.calculate(FlywheelShooter.getSelectedSensorVelocity());
    //     System.out.println("Arbitrary FF Value: " + kF);
    //     FlywheelShooter.set(ControlMode.Velocity, speed, DemandType.ArbitraryFeedForward, kF);
    // }

    public void setWheelVelocity(double speed) {
        FlywheelShooter.set(ControlMode.Velocity, speed);
    }

    public void setFlywheelMotor(double speed) {
        FlywheelShooter.set(ControlMode.PercentOutput, speed);
    }
}
