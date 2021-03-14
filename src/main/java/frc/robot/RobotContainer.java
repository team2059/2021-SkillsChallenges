/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.commands.AutoHelpers.AutoShootTester;
import frc.robot.commands.AutoHelpers.AutoShoot_V2;
import frc.robot.commands.AutoHelpers.LoadNextBall;
import frc.robot.subsystems.*;
import frc.robot.TrajectoryLoader;
import edu.wpi.first.wpilibj.controller.PIDController;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static Joystick driveJS = new Joystick(0);
  public static ButtonBox buttonBox = new ButtonBox(1);
  public static Joystick testJS = new Joystick(2);

  private Conveyor m_Conveyor = new Conveyor();
  private static Turret m_Turret = new Turret();
  private static Shooter m_Shooter = new Shooter();
  private static DriveBase m_DriveBase = new DriveBase();
  private BallElevator m_BallElevator = new BallElevator();

  public static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  public static NetworkTable limelightBall = NetworkTableInstance.getDefault().getTable("limelight-ball");

  public static Shooter getShooter() {
    return m_Shooter;
  }

  public static DriveBase getDrive() {
    return m_DriveBase;
  }

  public static Turret getTurret() {
    return m_Turret;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Default Commands
    // m_Turret.setDefaultCommand(new TurretRotate(m_Turret));
    m_DriveBase.setDefaultCommand(new Drive(m_DriveBase));
    // m_BallElevator.setDefaultCommand(new LoadNextBall(m_BallElevator));
  }

  public Command getAutonomousCommand() throws IOException {

    // limelightBall.getEntry("pipeline").setNumber(1);

    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/Complete.wpilib.json");

//     System.out.println("Is Red Path? " + isRedPath());

//     if (isRedPath()) {
//       trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/GalacticSearchA/AR.wpilib.json");
//     } else {
//       trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/GalacticSearchA/AB.wpilib.json");
//     }


    Trajectory bouncePath = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

    RamseteCommand ramseteCommand = new RamseteCommand(
      bouncePath,
        m_DriveBase::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_DriveBase::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_DriveBase::tankDriveVolts,
        m_DriveBase
    );

    m_DriveBase.reset();
    // Reset odometry to the starting pose of the trajectory.
    m_DriveBase.resetOdometry(bouncePath.getInitialPose());

    ParallelCommandGroup bounceCollect = new ParallelCommandGroup(
      new PIDRotateTurret(m_Turret, 20000),
      new InstantCommand(() -> m_Conveyor.toggleCollector()),
      new ParallelCommandGroup(
                    new SetCollector(m_Conveyor, .75),
                    new LoadNextBall(m_BallElevator, .75)),
      ramseteCommand
    );

    SequentialCommandGroup finalCommand = new SequentialCommandGroup(
      bounceCollect.withTimeout(45),
      new InstantCommand(() -> System.out.println("Finished Ramsete")),
      new InstantCommand(() -> m_DriveBase.tankDriveVolts(0, 0)),
      new InstantCommand(() -> m_Conveyor.toggleCollector()),
      new PIDRotateTurret(m_Turret, 20000),
      new AutoShoot_V2(m_Turret, m_Shooter, m_BallElevator, 3.85, m_Conveyor),
      new PIDVelocityShooter(m_Shooter, 0)
    );

    // Run path following command, then stop at the end.
    return finalCommand.andThen(() -> System.out.println("Finished Auto Sequence"));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(driveJS, 1)
            .whileHeld(new ParallelCommandGroup(
                    new SetCollector(m_Conveyor, .75),
                    new LoadNextBall(m_BallElevator, .75)
            )).whenReleased(() -> m_Conveyor.setCollectorMotor(0));

    new JoystickButton(driveJS, 2)
            .whenPressed(() -> m_Conveyor.toggleCollector());

    new JoystickButton(driveJS, 3)
            .whenPressed(() -> m_Conveyor.setCollectorAgitator(DoubleSolenoid.Value.kReverse))
            .whenReleased(() -> m_Conveyor.setCollectorAgitator(DoubleSolenoid.Value.kForward));

    new JoystickButton(driveJS, 4)
            .whileHeld(new ParallelCommandGroup(
                    new SetBallElevator(m_BallElevator,-.3),
                    new SetFlywheel(m_Shooter, -.3)
            )).whenReleased(() -> m_BallElevator.setBallElevatorMotor(0));



    new JoystickButton(driveJS, 7)
            .whileHeld(() -> m_Turret.setTurretRotatorMotor(.5))
            .whenReleased(() -> m_Turret.setTurretRotatorMotor(0));

    new JoystickButton(driveJS, 8)
            .whileHeld(() -> m_Turret.setTurretRotatorMotor(-.5))
            .whenReleased(() -> m_Turret.setTurretRotatorMotor(0));

    new JoystickButton(driveJS, 11)
            .whenPressed(() -> m_Shooter.setHoodPosition(0))
            .whenReleased(() -> m_Shooter.setHoodMotor(0));

    new JoystickButton(driveJS, 12)
            .whenPressed(() -> m_Shooter.setHoodPosition(15))
            .whenReleased(() -> m_Shooter.setHoodMotor(0));

//     new JoystickButton(driveJS, 5)
//             .whileHeld(() -> m_Climber.setArmPosition(1000))
//             .whenReleased(() -> m_Climber.setClimberMotor(0));

    /*
    16 ON = WINCH
    16 OFF = LIFT
    17 ON = AUTO
    17 OFF = MAN

    BUTTON BOX - INPUT 3
     */

    /*
     * 10 ft - 3.87 Hood Height
     * 17 ft - 4.5ISH Hood Height
     * SHOOT
    * */
    new JoystickButton(buttonBox, 1)
            .whileHeld(new ConditionalCommand(
                    new ConditionalCommand(
                            new AutoShootTester(m_Turret, m_Shooter, m_BallElevator, 4.5, m_Conveyor).andThen(new PIDHoodSetPostion(m_Shooter, 0)),
                            new AutoShootTester(m_Turret, m_Shooter, m_BallElevator, 3.85, m_Conveyor).andThen(new PIDHoodSetPostion(m_Shooter, 0)),
                            getDistanceSelector()
                    ),
                new PIDVelocityShooter(m_Shooter, 15000),
                getShooterSelector()))
            .whenReleased(() -> m_Shooter.setFlywheelMotor(0));
    /* LOAD */
    new JoystickButton(buttonBox, 2)
            .whileHeld(new ParallelCommandGroup(
                    new SetCollector(m_Conveyor, .5),
                    new LoadNextBall(m_BallElevator, .75)
            )).whenReleased(() -> m_Conveyor.setCollectorMotor(0));

    /* TURRET LEFT */
    new JoystickButton(buttonBox, 7)
          .whileHeld(new ConditionalCommand(
                new PIDRotateTurret(m_Turret, 0),
                new TurretRotate(m_Turret, -.7),
                getShooterSelector()))
          .whenReleased(() -> m_Turret.setTurretRotatorMotor(0));

    /* TURRET RIGHT */
    new JoystickButton(buttonBox, 8)
            .whileHeld(new ConditionalCommand(
                new PIDRotateTurret(m_Turret, 30000),
                new TurretRotate(m_Turret, .7),
                getShooterSelector()))
            .whenReleased(() -> m_Turret.setTurretRotatorMotor(0));

    /* TURRET CENTER */
    new JoystickButton(buttonBox, 9)
            .whenPressed(new PIDRotateTurret(m_Turret, 21290))
            .whenReleased(() -> m_Turret.setTurretRotatorMotor(0));

    /* COLLECTOR */
    new JoystickButton(buttonBox, 10)
            .whileHeld(new ParallelCommandGroup(
                    new SetCollector(m_Conveyor, .5),
                    new LoadNextBall(m_BallElevator, .75)
            )).whenReleased(() -> m_BallElevator.setBallElevatorMotor(0));

//        new JoystickButton(buttonBox, 10)
//            .whileHeld(new SetCollector(m_Conveyor, .5))
//                .whenReleased(() -> m_Conveyor.setCollectorMotor(0));

    /* SHUTTER UP - 11 */
    new JoystickButton(buttonBox, 11)
            .whenPressed(new PIDHoodSetPostion(m_Shooter, 3.85))
            .whenReleased(() -> m_Shooter.setHoodMotor(0));

    /* SHUTTER UP - 12 */
    new JoystickButton(buttonBox, 11)
            .whenPressed(new PIDHoodSetPostion(m_Shooter, 0))
            .whenReleased(() -> m_Shooter.setHoodMotor(0));

//     /* CLIMB LEFT */
//     new JoystickButton(buttonBox, 13)
//             .whileHeld(() -> m_Climber.setTrunionMotor(-.75))
//             .whenReleased(() -> m_Climber.setTrunionMotor(0));

//     /* CLIMB RIGHT */
//     new JoystickButton(buttonBox, 14)
//             .whileHeld(() -> m_Climber.setTrunionMotor(.75))
//             .whenReleased(() -> m_Climber.setTrunionMotor(0));

//     /* CLIMB UP */
//     new JoystickButton(buttonBox, 15)
//             .whileHeld(new ConditionalCommand(
//                 new Winch(m_Climber, .75),
//                 new Arm(m_Climber, -.25),
//                 getClimberSelector()));


//     /* CLIMB DOWN */
//     new JoystickButton(buttonBox, 16)
//             .whileHeld(new ConditionalCommand(
//                     new Winch(m_Climber, 0),
//                     new Arm(m_Climber, -.03),
//                     getClimberSelector()));

  }

  private BooleanSupplier getShooterSelector() {
    return () -> buttonBox.getRawButton(18);
  }

  private BooleanSupplier getClimberSelector() {
    return () -> buttonBox.getRawButton(17);
  }

  private BooleanSupplier getDistanceSelector() {
    return () -> buttonBox.getRawButton(19);
  }

  private boolean isRedPath() {
    return limelightBall.getEntry("tx").getDouble(0.0) <= -21;
  }

}
