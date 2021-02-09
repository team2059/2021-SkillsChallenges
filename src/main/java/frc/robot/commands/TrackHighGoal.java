package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TrackHighGoal extends CommandBase {
    private final Turret m_Turret;
    double x_error;

    public TrackHighGoal(Turret turret) {
        this.m_Turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //read values periodically
        x_error = m_Turret.getTurretLimelight().getHorizontalOffset();
        double correctionP = .2;

        if (Math.abs(x_error) > .2){
            if (Math.abs(x_error*correctionP) > .05) {
//                SmartDashboard.putNumber("Power Sent", Math.signum(x_error*correctionP));
                m_Turret.setTurretRotatorMotor(Math.signum(x_error*correctionP * .5));
            } else if(Math.abs(x_error*correctionP) < .05) {
//                SmartDashboard.putNumber("Power Sent", Math.signum(x_error*correctionP));
                m_Turret.setTurretRotatorMotor(Math.signum(x_error*correctionP * .5));
            } else {
//                SmartDashboard.putNumber("Power Sent", x_error*correctionP);
                m_Turret.setTurretRotatorMotor(x_error*correctionP);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_Turret.setTurretRotatorMotor(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(x_error) < 1;
    }
}