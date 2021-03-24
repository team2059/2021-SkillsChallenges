package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Turret;

public class PIDTrackHighGoal extends PIDCommand {
    private Turret m_Turret;

    public PIDTrackHighGoal(Turret turret) {

        super(
                new PIDController(TurretConstants.RotatekP, TurretConstants.RotatekI, TurretConstants.RotatekD),
                turret::getHorizontalOffset,
                0,
                output -> turret.setTurretRotatorMotor(-(output + .05)),
                turret
        );
        this.m_Turret = turret;

        getController().enableContinuousInput(-30, 30);
        getController().setTolerance(TurretConstants.offsetTolerance, TurretConstants.offsetVelocityTolerance);
    }

    @Override
    public void initialize() {
        System.out.println("Starting Vision Tracking");
    }

    @Override
    public void end(boolean isInteruppted) {
        this.m_Turret.setTurretRotatorMotor(0);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint() || RobotContainer.limelight.getEntry("tv").getDouble(0.0) == 0.0;
    }
}
