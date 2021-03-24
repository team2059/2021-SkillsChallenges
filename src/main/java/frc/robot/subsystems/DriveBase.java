package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class DriveBase extends SubsystemBase {

  CANSparkMax leftFront = new CANSparkMax(DriveConstants.leftMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftMid = new CANSparkMax(DriveConstants.leftPrimarySlave, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftBack = new CANSparkMax(DriveConstants.leftSecondarySlave, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightFront = new CANSparkMax(DriveConstants.rightMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightMid = new CANSparkMax(DriveConstants.rightPrimarySlave, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightBack = new CANSparkMax(DriveConstants.rightSecondarySlave, CANSparkMaxLowLevel.MotorType.kBrushless);

  public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-ball");

  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(leftFront, leftMid, leftBack);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(rightFront, rightMid, rightBack);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  private final CANEncoder m_leftEncoder = leftFront.getEncoder();

  // The right-side drive encoder
  private final CANEncoder m_rightEncoder = rightFront.getEncoder();

  // The gyro sensor
  private final Gyro m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveBase() {
    leftFront.restoreFactoryDefaults();
    leftMid.restoreFactoryDefaults();
    leftBack.restoreFactoryDefaults();

    rightFront.restoreFactoryDefaults();
    rightMid.restoreFactoryDefaults();
    rightBack.restoreFactoryDefaults();

    leftFront.setIdleMode(IdleMode.kCoast);
    leftMid.setIdleMode(IdleMode.kCoast);
    leftBack.setIdleMode(IdleMode.kCoast);
    rightFront.setIdleMode(IdleMode.kCoast);
    rightMid.setIdleMode(IdleMode.kCoast);
    rightBack.setIdleMode(IdleMode.kCoast);

    System.out.println("All Motors set to Coast");

    m_gyro.reset();

    m_leftEncoder.setPositionConversionFactor(DriveConstants.kLinearDistancePerMotorRotation);
    m_rightEncoder.setPositionConversionFactor(DriveConstants.kLinearDistancePerMotorRotation);
    m_rightEncoder.setVelocityConversionFactor(DriveConstants.kLinearDistancePerMotorRotation / 60);
    m_leftEncoder.setVelocityConversionFactor(DriveConstants.kLinearDistancePerMotorRotation / 60);

    rightFront.setInverted(true);
    rightMid.setInverted(true);
    rightBack.setInverted(true);


    System.out.println(m_leftEncoder.getPositionConversionFactor());
    System.out.println(m_leftEncoder.getVelocityConversionFactor());
    System.out.println(m_leftEncoder.getCountsPerRevolution());
    System.out.println(m_leftEncoder.getMeasurementPeriod());

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getPosition(),
                      m_rightEncoder.getPosition());

    SmartDashboard.putNumber("REncoder Pos", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("REncoder Vel", m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("LEncoder Pos", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("LEncoder Vel", m_leftEncoder.
    getVelocity());

    // System.out.println("Gyro Angle " + m_gyro.getAngle());
    // System.out.println("Gyro Heading " + getHeading());

    SmartDashboard.putNumber("Current Left Speed", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Current Right Speed", m_rightEncoder.getVelocity());
  }

  public void setIdleMode(IdleMode mode) {
    leftFront.setIdleMode(mode);
    leftMid.setIdleMode(mode);
    leftBack.setIdleMode(mode);
    rightFront.setIdleMode(mode);
    rightMid.setIdleMode(mode);
    rightBack.setIdleMode(mode);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    // System.out.println("fws power" + fwd);
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    // System.out.println(leftVolts + " " + rightVolts);
    // TODO: UnClamp Voltage
    // leftVolts = MathUtil.clamp(leftVolts, -2, 2);
    // rightVolts = MathUtil.clamp(rightVolts, -2, 2);
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
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
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public void reset() {
    m_gyro.reset();
    leftFront.getEncoder().setPosition(0);
    rightFront.getEncoder().setPosition(0);
  }

  public void drive(){

    double speed = RobotContainer.driveJS.getRawAxis(0)*.5;
    double rotation = RobotContainer.driveJS.getRawAxis(1)*.5;

    // if (RobotContainer.driveJS.getRawButton(1)) {
    //   m_drive.arcadeDrive(speed, -rotation);
    // } else {
      m_drive.arcadeDrive(speed, rotation);
    // }
    
  }

  
  public double getBallOffset() {
    return table.getEntry("tx").getDouble(0.0);
  }

  public double getGyroAngle() {
    return m_gyro.getAngle();
  }
  
  public double getStraightDistance() {
    return (leftFront.getEncoder().getPosition() + rightBack.getEncoder().getPosition()) / 2;
  }

}
