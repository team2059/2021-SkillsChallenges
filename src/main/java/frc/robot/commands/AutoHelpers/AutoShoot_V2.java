package frc.robot.commands.AutoHelpers;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.PIDHoodSetPostion;
import frc.robot.commands.PIDTrackHighGoal;
import frc.robot.commands.PIDVelocityShooter;
import frc.robot.subsystems.BallElevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class AutoShoot_V2  extends ParallelCommandGroup {

    public AutoShoot_V2(Turret turret, Shooter shooter, BallElevator ballElevator, double hoodPosition) {
        addCommands(
                new PIDTrackHighGoal(turret),
                new PIDVelocityShooter(shooter, 15000),
                new PIDHoodSetPostion(hoodPosition, shooter),
                new AutoLoad(ballElevator, .5)
        );
    }
}
