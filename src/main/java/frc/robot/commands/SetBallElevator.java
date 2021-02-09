package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallElevator;
import frc.robot.subsystems.Conveyor;

public class SetBallElevator extends CommandBase {
    private final BallElevator m_BallElevator;
    private double speed;

    public SetBallElevator(BallElevator subsystem, double speed) {
        this.m_BallElevator = subsystem;
        this.speed = speed;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Starting Ball Elevator");
    }

    @Override
    public void execute() {
        m_BallElevator.setBallElevatorMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_BallElevator.setBallElevatorMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
