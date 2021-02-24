/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {

  CANSparkMax leftFront;
  CANSparkMax leftBack;
  CANSparkMax rightFront;
  CANSparkMax rightBack;

  SpeedControllerGroup leftMotors;
  SpeedControllerGroup rightMotors;

  DifferentialDrive dDrive;

  AHRS navX;

  CANEncoder leftEncoder;
  CANEncoder rightEncoder;

  /**
   * Creates a new DriveBase.
   */
  public DriveBase() {
    leftFront = new CANSparkMax(Constants.leftForwardMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftBack = new CANSparkMax(Constants.leftBackMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightFront = new CANSparkMax(Constants.rightForwardMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightBack = new CANSparkMax(Constants.rightBackMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    navX = new AHRS(SPI.Port.kMXP);

    leftEncoder = leftFront.getEncoder();
    rightEncoder = rightFront.getEncoder();

    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    leftFront.setInverted(false);
    rightFront.setInverted(true);

    leftMotors = new SpeedControllerGroup(leftFront, leftBack);
    rightMotors = new SpeedControllerGroup(rightFront, rightBack);

    dDrive = new DifferentialDrive(leftMotors, rightMotors);
  }

  public double getGyroAngle() {
    return navX.getAngle();
  }

  public double getLeftTicks() {
    return leftEncoder.getPosition();
  }

  public double getRightTicks() {
    return rightEncoder.getPosition();
  }

  public double getDistance() {
    return (getLeftTicks() + getRightTicks()) / 2;
  }

  public void setMotors(double speed) {
    leftMotors.set(speed);
    rightMotors.set(speed);
  }

  public void tankDrive(double left, double right) {
    leftMotors.set(left);
    rightMotors.set(right);
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    dDrive.arcadeDrive(xSpeed, zRotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
