package frc.robot.commands.AutoHelpers;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.PIDHoodSetPostion;
import frc.robot.commands.PIDTrackHighGoal;
import frc.robot.commands.PIDVelocityShooter;
import frc.robot.commands.TurretRotate;
import frc.robot.subsystems.BallElevator;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class AutoShootTester  extends ParallelCommandGroup {

    public AutoShootTester(Turret turret, Shooter shooter, BallElevator ballElevator, double hoodPosition, Conveyor conveyor) {
        addCommands(
            new PIDTrackHighGoal(turret),
            new PIDVelocityShooter(shooter, 21000),
            new InstantCommand(() -> conveyor.setAgitator(.55)),
            new PIDHoodSetPostion(shooter, hoodPosition),
            new AutoLoad(ballElevator, .35)
        );
    }
}