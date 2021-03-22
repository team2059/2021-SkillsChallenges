package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class PIDVelocityShooter extends CommandBase {
    private final Shooter m_Shooter;
    private double setpoint;

    public PIDVelocityShooter(Shooter subsystem, int setpoint) {
        this.m_Shooter = subsystem;
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        System.out.println("Starting Velocity Shooter \n\n\n");
    }

    @Override
    public void execute() {
        m_Shooter.setTargetVeloctiy((int) this.setpoint);
        m_Shooter.setWheelVelocity(this.setpoint);
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