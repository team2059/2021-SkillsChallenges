package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Turret;

public class PIDRotateTurret extends PIDCommand {

    Turret m_Turret;
    double setpoint;

    public PIDRotateTurret(Turret turret, double setpoint){
        super(
                new PIDController(Constants.TurretConstants.EncoderRotatekP, Constants.TurretConstants.EncoderRotatekI, Constants.TurretConstants.EncoderRotatekD),
                turret::getTurretEncoderLocation,
                setpoint,
                output -> turret.setTurretRotatorMotor(output),
                turret
        );
        this.m_Turret = turret;
        this.setpoint = setpoint;
        System.out.println("Starting auto rotate");
//        getController().setTolerance(500, 0);
    }

    @Override
    public void initialize() {
        System.out.println("Starting auto rotate");
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint() ||  Math.abs(m_Turret.getTurretEncoderLocation() - setpoint) < 500;
    }
}
