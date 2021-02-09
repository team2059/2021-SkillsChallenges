package frc.robot.commands.AutoHelpers;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PIDHoodSetPostion;
import frc.robot.commands.PIDTrackHighGoal;
import frc.robot.commands.PIDVelocityShooter;
import frc.robot.subsystems.*;

public class AutoShoot extends SequentialCommandGroup {

    // TODO: Adjust the hood height dynamically

        public AutoShoot(Turret turret, Shooter shooter, BallElevator ballElevator, double hoodPosition) {
            addCommands(
                    new ParallelCommandGroup(
                            new PIDTrackHighGoal(turret),
                            new PIDVelocityShooter(shooter, 15000),
                            new PIDHoodSetPostion(hoodPosition, shooter),
                            new AutoLoad(ballElevator, .5)
                    ).withTimeout(6.5),
                     new PIDHoodSetPostion(0, shooter)
            );
        }
}
