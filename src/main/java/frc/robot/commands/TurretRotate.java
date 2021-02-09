package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Turret;

public class TurretRotate extends CommandBase {
    private final Turret m_Turret;
    private double speed;

    public TurretRotate(Turret subsystem, double speed) {
        this.m_Turret = subsystem;
        this.speed = speed;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Starting Turret");
    }

    @Override
    public void execute() {

        m_Turret.setTurretRotatorMotor(speed);
//        m_Turret.rotateTurret();
    }

    @Override
    public void end(boolean interrupted) {
        m_Turret.setTurretRotatorMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}