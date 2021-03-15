/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.RamseteCmd;
import frc.robot.subsystems.DrivebaseSubsytem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public static final Joystick DRIVE_JOYSTICK = new Joystick(0);

    public final static DrivebaseSubsytem m_robotDrive = new DrivebaseSubsytem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(
                        () -> m_robotDrive.arcadeDrive(DRIVE_JOYSTICK.getRawAxis(2), DRIVE_JOYSTICK.getRawAxis(1)*.60),
                        m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        m_robotDrive.setMaxOutput(.2);

        // Trajectory slalomTraj = new Trajectory();
        // Path slalomTrajPath = Filesystem.getDeployDirectory().toPath().resolve("paths/slalomPath/infinity.wpilib.json");

        // try {
        //     slalomTraj = TrajectoryUtil.fromPathweaverJson(slalomTrajPath);
        // } catch (IOException e) {
        //     // TODO Auto-generated catch block
        //     System.out.println("Could not find segment 1");
        // }

        // final Pose2d slalomInitPose = slalomTraj.getInitialPose();

        // m_robotDrive.resetOdometry(slalomTraj.getInitialPose());

        // RamseteCommand slalom = new RamseteCommand(
        //     slalomTraj,
        //     m_robotDrive::getPose,
        //     new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        //     new SimpleMotorFeedforward(DriveConstants.ksVolts,
        //                             DriveConstants.kvVoltSecondsPerMeter,
        //                             DriveConstants.kaVoltSecondsSquaredPerMeter),
        //     DriveConstants.kDriveKinematics,
        //     m_robotDrive::getWheelSpeeds,
        //     new PIDController(DriveConstants.kPDriveVel, 0, 0),
        //     new PIDController(DriveConstants.kPDriveVel, 0, 0),
        //     // RamseteCommand passes volts to the callback
        //     m_robotDrive::tankDriveVolts,
        //     m_robotDrive
        // );

        // m_robotDrive.resetOdometry(slalomTraj.getInitialPose());


        Trajectory seg1 = new Trajectory();
        Trajectory seg2 = new Trajectory();
        Trajectory seg3 = new Trajectory();
        Trajectory seg4 = new Trajectory();

        Path seg1Path = Filesystem.getDeployDirectory().toPath().resolve("paths/bouncePath/seg1.wpilib.json");
        Path seg2Path = Filesystem.getDeployDirectory().toPath().resolve("paths/bouncePath/seg2.wpilib.json");
        Path seg3Path = Filesystem.getDeployDirectory().toPath().resolve("paths/bouncePath/seg3.wpilib.json");
        Path seg4Path = Filesystem.getDeployDirectory().toPath().resolve("paths/bouncePath/seg4.wpilib.json");
        
        try {
            seg1 = TrajectoryUtil.fromPathweaverJson(seg1Path);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            System.out.println("Could not find segment 1");
        }
        try {
            seg2 = TrajectoryUtil.fromPathweaverJson(seg2Path);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            System.out.println("Could not find segment 2");
        }
        try {
            seg3 = TrajectoryUtil.fromPathweaverJson(seg3Path);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            System.out.println("Could not find segment 3");
        }
        try {
            seg4 = TrajectoryUtil.fromPathweaverJson(seg4Path);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            System.out.println("Could not find segment 4");
        }

        final Pose2d seg2InitPose = seg2.getInitialPose();
        final Pose2d seg3InitPose = seg3.getInitialPose();
        final Pose2d seg4InitPose = seg4.getInitialPose();

    SequentialCommandGroup bouncePath = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotDrive.setDriveIdleMode(IdleMode.kBrake)),
        new RamseteCmd(
            seg1,
            m_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                    DriveConstants.kvVoltSecondsPerMeter,
                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive
        ),
        new InstantCommand(() -> m_robotDrive.resetOdometry(seg2InitPose)),
        new RamseteCmd(
            seg2,
            m_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                    DriveConstants.kvVoltSecondsPerMeter,
                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive
        ),
        new InstantCommand(() -> m_robotDrive.resetOdometry(seg3InitPose)),
        new RamseteCmd(
            seg3,
            m_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                    DriveConstants.kvVoltSecondsPerMeter,
                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive
        ),
        new InstantCommand(() -> m_robotDrive.resetOdometry(seg4InitPose)),
        new RamseteCmd(
            seg4,
            m_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                    DriveConstants.kvVoltSecondsPerMeter,
                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive
        ),
        new InstantCommand(() -> m_robotDrive.setDriveIdleMode(IdleMode.kCoast))
    );

    // // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(seg1.getInitialPose());

    // Run path following command, then stop at the end.
    return bouncePath.andThen(() -> m_robotDrive.tankDriveVolts(0, 0)).andThen(() -> System.out.println("Finished"));
  }
}
