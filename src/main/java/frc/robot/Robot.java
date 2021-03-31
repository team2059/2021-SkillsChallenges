package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

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

      RobotContainer.limelightBall.getEntry("camMode").setNumber(1);
      RobotContainer.limelightBall.getEntry("ledMode").setNumber(1);

      RobotContainer.limelight.getEntry("camMode").setNumber(1);
      RobotContainer.limelight.getEntry("ledMode").setNumber(1);
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Compressor Current", compressor.getCompressorCurrent());

    SmartDashboard.putNumber("Estimated Distance", RobotContainer.getTurret().getDistanceToTarget());

    // if (RobotContainer.buttonBox.getRawButton(20)) {
    //   RobotContainer.limelight.getEntry("ledMode").setNumber(1);
    // } else {
    //   RobotContainer.limelight.getEntry("ledMode").setNumber(3);
    // }
    SmartDashboard.putBoolean("Ten Foot Shot", Constants.FieldConstants.isTenFootShot);
  }

  @Override
  public void disabledInit() {
    try {

      RobotContainer.getShooter().setHoodMode(IdleMode.kCoast);
      RobotContainer.limelightBall.getEntry("pipeline").setNumber(1);

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
      RobotContainer.limelightBall.getEntry("pipeline").setNumber(1);
      CrashTracker.logAutoInit();
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();

      if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
      }
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      try {
        throw t;
      } catch (Throwable e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
  }

  @Override
  public void autonomousPeriodic() {
  }


  @Override
  public void teleopInit() {
    try {
      CrashTracker.logTeleopInit();

      // RobotContainer.limelightBall.getEntry("camMode").setNumber(1);
      // RobotContainer.limelightBall.getEntry("ledMode").setNumber(1);

      // RobotContainer.limelight.getEntry("ledMode").setNumber(1);

      RobotContainer.getDrive().resetEncoders();
      RobotContainer.getShooter().setHoodMode(IdleMode.kBrake);


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
