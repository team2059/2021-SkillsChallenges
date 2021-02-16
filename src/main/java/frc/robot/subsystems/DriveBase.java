package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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

    private final Gyro gyro = new AHRS(SPI.Port.kMXP);

    private final double ticksPerFoot = 142.2;

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24));
    DifferentialDriveOdometry m_odometry;

    SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftMaster, leftPrimarySlave, leftSecondarySlave);
    SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightMaster, rightPrimarySlave, rightSecondarySlave);

    DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

    // The left-side drive encoder
    private final CANEncoder m_leftEncoder = leftMaster.getEncoder();

    // The right-side drive encoder
    private final CANEncoder m_rightEncoder = rightMaster.getEncoder();

    PIDController leftPIDController = new PIDController(0.747, 0, 0);
    PIDController rightPIDController = new PIDController(0.747, 0, 0);

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.276, .0358, 0.0153);

    Pose2d pose = new Pose2d();

    public DriveBase() {
        super("DriveBase");

        leftMaster.restoreFactoryDefaults();
        rightMaster.restoreFactoryDefaults();
        leftPrimarySlave.restoreFactoryDefaults();
        leftSecondarySlave.restoreFactoryDefaults();
        rightPrimarySlave.restoreFactoryDefaults();
        rightSecondarySlave.restoreFactoryDefaults();

        leftMaster.setIdleMode(IdleMode.kCoast);
        rightMaster.setIdleMode(IdleMode.kCoast);
        leftPrimarySlave.setIdleMode(IdleMode.kCoast);
        leftSecondarySlave.setIdleMode(IdleMode.kCoast);
        rightPrimarySlave.setIdleMode(IdleMode.kCoast);
        rightSecondarySlave.setIdleMode(IdleMode.kCoast);

        gyro.reset();

        leftPrimarySlave.follow(leftMaster);
        leftSecondarySlave.follow(leftMaster);
        rightPrimarySlave.follow(rightMaster);
        rightPrimarySlave.follow(rightMaster);

        leftMaster.setInverted(true);
        leftPrimarySlave.setInverted(true);
        leftSecondarySlave.setInverted(true);
        rightMaster.setInverted(false);
        rightPrimarySlave.setInverted(false);
        rightSecondarySlave.setInverted(false);

        leftMaster.getEncoder().setPositionConversionFactor(DriveConstants.kLinearDistancePerMotorRotation);
        rightMaster.getEncoder().setPositionConversionFactor(DriveConstants.kLinearDistancePerMotorRotation);
        leftMaster.getEncoder().setPosition(0);
        rightMaster.getEncoder().setPosition(0);

        differentialDrive.setSafetyEnabled(false);
        m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    }

    @Override
    public void periodic() {
        m_odometry.update(gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
        SmartDashboard.putNumber("Gyro Angle", -gyro.getAngle());
        SmartDashboard.putNumber("Gyro Heading", getHeading());
        SmartDashboard.putNumber("Left Ticks", leftMaster.getEncoder().getPosition());
        SmartDashboard.putNumber("Right Ticks", rightMaster.getEncoder().getPosition());
        SmartDashboard.putNumber("Ball Offset", getBallOffset());
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
        leftMaster.getEncoder().setPosition(0);
        rightMaster.getEncoder().setPosition(0);
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

    public void resetEncoders() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public CANEncoder getLeftEncoder() {
        return m_leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public CANEncoder getRightEncoder() {
        return m_rightEncoder;
    }

    /**
     * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        differentialDrive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -gyro.getRate();
    }

    @Override
    public void update(RobotState robotState) {

    }
}