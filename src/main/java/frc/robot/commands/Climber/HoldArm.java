package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class HoldArm extends CommandBase {
    private final Climber m_Climber;

    public HoldArm(Climber subsystem) {
        m_Climber = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (m_Climber.getArmLocation() < -50) {
            m_Climber.setClimberMotor(-.1);
        }
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
