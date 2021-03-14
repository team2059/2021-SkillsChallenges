package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoHelpers.AutoLoad;
import frc.robot.commands.PIDDrive.StraightDrive;
import frc.robot.subsystems.*;

public class InitLineAuto extends SequentialCommandGroup {
    public InitLineAuto(DriveBase drive, Turret turret, Shooter shooter, Conveyor conveyor, BallElevator ballElevator) {
        addCommands(
                new PIDRotateTurret(turret, 21290),
                new ParallelCommandGroup(
                        new PIDTrackHighGoal(turret),
                        new PIDVelocityShooter(shooter, 15000),
                        new PIDHoodSetPostion(shooter, 0),
                        new AutoLoad(ballElevator, .5)
                ).withTimeout(6.5),
//                new StraightDrive(drive, -5)
                new ParallelCommandGroup(
                        new StraightDrive(drive, 10),
                        new SetCollector(conveyor, .7)
                )
//                        .withTimeout(3),
//                new ParallelCommandGroup(
//                        new PIDTrackHighGoal(turret),
//                        new PIDVelocityShooter(shooter, 15000),
//                        new PIDHoodSetPostion(4.023, shooter),
//                        new AutoLoad(ballElevator, .5)
//                )
//                new ParallelCommandGroup(
//                        new PIDRotateTurret(turret, 30000),
//                        new PIDVelocityShooter(shooter, 15000)
//                ),
//                new PIDTrackHighGoal(turret),
//                new ParallelCommandGroup(
//                        new PIDVelocityShooter(shooter, 15000),
//                        new AutoShoot(ballElevator)
//                )
        );

                // Drive forward the specified distance
//                new DriveDistance(AutoConstants.kAutoDriveDistanceInches, AutoConstants.kAutoDriveSpeed,
//                        drive),
//
//                // Release the hatch
//                new ReleaseHatch(hatch),
//
//                // Drive backward the specified distance
//                new DriveDistance(AutoConstants.kAutoBackupDistanceInches, -AutoConstants.kAutoDriveSpeed,
//                        drive));
    }
}
