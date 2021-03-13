package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class PIDHoodSetPostion extends PIDCommand {

    public PIDHoodSetPostion(double setpoint, Shooter shooter) {
        super(
                new PIDController(Constants.ShooterConstants.HoodkP, Constants.ShooterConstants.HoodkI, Constants.ShooterConstants.HoodkD),
                shooter::getHoodPosition,
                setpoint,
                output -> shooter.setHoodMotor(output),
                shooter
        );

        getController().setTolerance(0.05);
    }

    @Override
    public void initialize() {
        System.out.println("Shooting hood " + getController().getSetpoint());

        System.out.println("Starting Hood Position");
    }

    @Override
    public boolean isFinished() {
        return false;
        // return getController().atSetpoint();
    }
}
