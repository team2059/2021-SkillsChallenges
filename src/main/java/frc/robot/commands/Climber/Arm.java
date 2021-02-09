package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Arm extends CommandBase {
    private final Climber m_Climber;
    private double speed;

    public Arm(Climber subsystem, double speed) {
        m_Climber = subsystem;
        this.speed = speed;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_Climber.setClimberMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_Climber.setClimberMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
