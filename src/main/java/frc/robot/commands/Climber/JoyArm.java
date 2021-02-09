package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class JoyArm extends CommandBase {
    private final Climber m_Climber;
    private Joystick js;

    public JoyArm(Climber subsystem, Joystick js) {
        m_Climber = subsystem;
        this.js = js;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        SmartDashboard.putNumber("Joystick power", js.getRawAxis(1));
        m_Climber.setClimberMotor(js.getRawAxis(1));
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
