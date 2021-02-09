package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallElevator;
import frc.robot.subsystems.Shooter;

public class SetFlywheel extends CommandBase {
    private final Shooter m_Shooter;
    private double speed;

    public SetFlywheel(Shooter subsystem, double speed) {
        this.m_Shooter = subsystem;
        this.speed = speed;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Starting Flywheel");
    }

    @Override
    public void execute() {
        m_Shooter.setFlywheelMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_Shooter.setFlywheelMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
