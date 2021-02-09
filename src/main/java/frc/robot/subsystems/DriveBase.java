package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import hhCore.config.RobotState;
import hhCore.subsystems.HHSubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveBase extends HHSubsystemBase {

    CANSparkMax leftMaster = new CANSparkMax(DriveConstants.leftMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax rightMaster = new CANSparkMax(DriveConstants.rightMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax leftPrimarySlave = new CANSparkMax(DriveConstants.leftPrimarySlave, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax leftSecondarySlave = new CANSparkMax(DriveConstants.leftSecondarySlave, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax rightPrimarySlave = new CANSparkMax(DriveConstants.rightPrimarySlave, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax rightSecondarySlave = new CANSparkMax(DriveConstants.rightSecondarySlave, CANSparkMaxLowLevel.MotorType.kBrushless);

    public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-ball");

    AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final double ticksPerFoot = 142.2;

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24));
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), new Pose2d(0, 0, getHeading()));

    SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftMaster, leftPrimarySlave, leftSecondarySlave);
    SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightMaster, rightPrimarySlave, rightSecondarySlave);

    DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

    PIDController leftPIDController = new PIDController(0.747, 0, 0);
    PIDController rightPIDController = new PIDController(0.747, 0, 0);

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.276, .0358, 0.0153);

    Pose2d pose = new Pose2d();

    public DriveBase() {

        super("DriveBase");

        leftPrimarySlave.follow(leftMaster);
        leftSecondarySlave.follow(leftMaster);
        rightPrimarySlave.follow(rightMaster);
        rightPrimarySlave.follow(rightMaster);

        leftMaster.setInverted(false);
        rightMaster.setInverted(true);

        leftMaster.getEncoder().setPositionConversionFactor(42);
        rightMaster.getEncoder().setPositionConversionFactor(42);
        leftMaster.getEncoder().setPosition(0);
        rightMaster.getEncoder().setPosition(0);

        differentialDrive.setSafetyEnabled(false);

        gyro.reset();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    public double getGyroAngle() {
        return gyro.getAngle();
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                leftMaster.getEncoder().getVelocity() / DriveConstants.kGearRatio * 2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadiusInches) / 60,
                rightMaster.getEncoder().getVelocity() / DriveConstants.kGearRatio * 2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadiusInches) / 60
        );
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public Pose2d getPose() {
        return pose;
    }

    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }

    public PIDController getLeftPIDController() {
        return leftPIDController;
    }

    public PIDController getRightPIDController() {
        return rightPIDController;
    }

    public void setOutputVolts(double leftVolts, double rightVolts) {
        leftMaster.set(leftVolts / 12);
        rightMaster.set(rightVolts / 12);
    }

    public void reset() {
        gyro.reset();
        odometry.resetPosition(new Pose2d(), getHeading());
        leftMaster.getEncoder().setPosition(0);
        rightMaster.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        pose = odometry.update(getHeading(), leftMaster.getEncoder().getPosition()/ DriveConstants.kGearRatio * 2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadiusInches), rightMaster.getEncoder().getPosition()/ DriveConstants.kGearRatio * 2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadiusInches));
        SmartDashboard.putNumber("Heading", getHeading().getDegrees());
        SmartDashboard.putNumber("Left Ticks", leftMaster.getEncoder().getPosition());
        SmartDashboard.putNumber("Right Ticks", rightMaster.getEncoder().getPosition());
        SmartDashboard.putNumber("Straight Distance", getStraightDistance());
        SmartDashboard.putNumber("Ball Offset", getBallOffset());
    }

    public void drive(){
        differentialDrive.arcadeDrive(RobotContainer.driveJS.getRawAxis(0), RobotContainer.driveJS.getRawAxis(1));
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        System.out.println(String.format("xSpeed is %f, zRotation is %f", xSpeed, zRotation));
        differentialDrive.arcadeDrive(-xSpeed, zRotation);
    }

    public void tankDrive(double l, double r) {

        l = MathUtil.clamp(l, -.5, .5);
        r = MathUtil.clamp(r, -.5, .5);

        leftMotors.set(l);
        rightMotors.set(r);
    }

    public double getBallOffset() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getStraightDistance() {
        return (leftMaster.getEncoder().getPosition()/ticksPerFoot + leftMaster.getEncoder().getPosition()/ticksPerFoot) / 2;
    }


    @Override
    public void update(RobotState robotState) {

    }
}