package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Winch extends CommandBase {
    private final Climber m_Climber;
    private double speed;

    public Winch(Climber subsystem, double speed) {
        m_Climber = subsystem;
        this.speed = speed;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_Climber.setWinchMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_Climber.setWinchMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
