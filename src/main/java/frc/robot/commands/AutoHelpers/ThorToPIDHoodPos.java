package frc.robot.commands.AutoHelpers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallElevator;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class ThorToPIDHoodPos extends CommandBase {
    private double setPoint;
    private Shooter m_Shooter;

    public ThorToPIDHoodPos(Shooter subsystem) {
        this.m_Shooter = subsystem;
    }

    @Override
    public void initialize() {
        System.out.println("Starting Ball Elevator");
    }

    @Override
    public void execute() {
        m_Shooter.setHoodPosition(m_Shooter.thorToHoodPos());
    }

    @Override
    public void end(boolean interrupted) {
        m_Shooter.setHoodMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
