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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.AutoHelpers.AutoShoot_V2;
import frc.robot.commands.AutoHelpers.LoadNextBall;
import frc.robot.subsystems.*;

import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static Joystick driveJS = new Joystick(0);
  public static ButtonBox buttonBox = new ButtonBox(1);
  public static Joystick testJS = new Joystick(2);

  private Conveyor m_Conveyor = new Conveyor();
  private Turret m_Turret = new Turret();
  private static Shooter m_Shooter = new Shooter();
  private DriveBase m_DriveBase = new DriveBase();
  private BallElevator m_BallElevator = new BallElevator();

  public static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  public static NetworkTable limelightBall = NetworkTableInstance.getDefault().getTable("limelight-ball");


  public static Shooter getShooter() {
    return m_Shooter;
  }


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Default Commands
//    m_Turret.setDefaultCommand(new TurretRotate(m_Turret));
    m_DriveBase.setDefaultCommand(new Drive(m_DriveBase));
//    m_BallElevator.setDefaultCommand(new LoadNextBall(m_BallElevator));
  }

  public Command getAutonomousCommand() {
//    TrajectoryConfig config = new TrajectoryConfig(
//            Units.feetToMeters(.01), Units.feetToMeters(.01));
//    config.setKinematics(m_DriveBase.getKinematics());
//
//    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
//      Arrays.asList(new Pose2d(),
//              new Pose2d(.5, 0, new Rotation2d())),
////                    new Pose2d(-1.35, 0, new Rotation2d())),
//      config
//);
//
//    RamseteCommand command = new RamseteCommand(
//            trajectory,
//            m_DriveBase::getPose,
//            new RamseteController(2, .7),
//            m_DriveBase.getFeedforward(),
//            m_DriveBase.getKinematics(),
//            m_DriveBase::getSpeeds,
//            m_DriveBase.getLeftPIDController(),
//            m_DriveBase.getRightPIDController(),
//            m_DriveBase::setOutputVolts,
//            m_DriveBase
//    );
//
//    return command.andThen(() -> m_DriveBase.setOutputVolts(0, 0)).beforeStarting(() -> System.out.println("Starting Ramsete Command"));
//    return new StraightDrive(m_DriveBase, -5);

    return new InitLineAuto(m_DriveBase, m_Turret, m_Shooter, m_Conveyor, m_BallElevator);
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
                    new SetCollector(m_Conveyor, .5),
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
            .whenPressed(new PIDHoodSetPostion(3.738, m_Shooter))
            .whenReleased(() -> m_Shooter.setHoodMotor(0));

    new JoystickButton(driveJS, 12)
            .whenPressed(new PIDHoodSetPostion(0, m_Shooter))
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
                            new AutoShoot_V2(m_Turret, m_Shooter, m_BallElevator, 4.5).andThen(new PIDHoodSetPostion(0, m_Shooter)),
                            new AutoShoot_V2(m_Turret, m_Shooter, m_BallElevator, 3.85).andThen(new PIDHoodSetPostion(0, m_Shooter)),
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
            .whenPressed(new PIDHoodSetPostion(3.85, m_Shooter))
            .whenReleased(() -> m_Shooter.setHoodMotor(0));

    /* SHUTTER UP - 12 */
    new JoystickButton(buttonBox, 11)
            .whenPressed(new PIDHoodSetPostion(0, m_Shooter))
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

}
