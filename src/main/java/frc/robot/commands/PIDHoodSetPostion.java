package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallElevator;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class PIDHoodSetPostion extends CommandBase {
    private double setPoint;
    private Shooter m_Shooter;

    public PIDHoodSetPostion(Shooter subsystem, double setpoint) {
        this.m_Shooter = subsystem;
        this.setPoint = setpoint;
    }

    @Override
    public void initialize() {
        System.out.println("Starting Ball Elevator");
    }

    @Override
    public void execute() {
        m_Shooter.setHoodPosition(this.setPoint);
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
