package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import hhCore.utils.CrashTracker;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Compressor compressor = new Compressor();

  Robot() {
    CrashTracker.logRobotStartup();
  }

  @Override
  public void robotInit() {
    try {
      CrashTracker.logRobotInit();

      compressor.setClosedLoopControl(true);

      m_robotContainer = new RobotContainer();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Compressor Current", compressor.getCompressorCurrent());

    if (RobotContainer.buttonBox.getRawButton(20)) {
      RobotContainer.limelight.getEntry("ledMode").setNumber(1);
    } else {
      RobotContainer.limelight.getEntry("ledMode").setNumber(3);
    }
    SmartDashboard.putBoolean("Ten Foot Shot", Constants.FieldConstants.isTenFootShot);
  }

  @Override
  public void disabledInit() {
    try {
      CrashTracker.logDisabled();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    try {
      CrashTracker.logAutoInit();
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();

      if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
      }
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void autonomousPeriodic() {
  }


  @Override
  public void teleopInit() {
    try {
      CrashTracker.logTeleopInit();

      RobotContainer.limelightBall.getEntry("camMode").setNumber(1);
      RobotContainer.limelightBall.getEntry("ledMode").setNumber(1);

      RobotContainer.limelight.getEntry("ledMode").setNumber(1);


      if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
      }
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void teleopPeriodic() {



    SmartDashboard.putBoolean("Ball Offset", Constants.FieldConstants.isTenFootShot);
  }

  @Override
  public void testInit() {
    try {
      CrashTracker.logTestInit();

      CommandScheduler.getInstance().cancelAll();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void testPeriodic() {
  }
}
