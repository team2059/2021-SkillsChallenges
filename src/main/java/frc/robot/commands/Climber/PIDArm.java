package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class PIDArm extends PIDCommand {

    Climber m_Climber;
    double setpoint;

    public PIDArm(Climber climber, double setpoint){
        super(
                new PIDController(Constants.ClimberConstants.ArmKp, Constants.ClimberConstants.ArmkI, Constants.ClimberConstants.ArmkD),
                climber::getArmLocation,
                setpoint,
                output -> climber.setClimberMotor(output),
                climber
        );
        this.m_Climber = climber;
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        System.out.println("Starting arm pid");
    }

    @Override
    public void execute() {
        System.out.println("Executing arm pid");
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint(); // ||  Math.abs(m_Climber.getArmLocation() - setpoint) < 500;
    }
}
