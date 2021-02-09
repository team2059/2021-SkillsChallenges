package frc.robot.commands.AutoHelpers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallElevator;

public class LoadNextBall extends CommandBase {
    private final BallElevator m_BallElevator;
    private double speed;

    public LoadNextBall(BallElevator subsystem, double speed) {
        this.m_BallElevator = subsystem;
        this.speed = speed;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Starting Auto Loader");
    }

    @Override
    public void execute() {
        m_BallElevator.autoLoader(speed);
//        if (!m_BallElevator.isBallInChamber()) {
//            m_BallElevator.setBallElevatorMotor(speed);
//            SmartDashboard.putBoolean("Next Shot Ready", false);
//        } else {
//            SmartDashboard.putBoolean("Next Shot Ready", true);
//        }
    }

    @Override
    public void end(boolean interrupted) {
        m_BallElevator.setBallElevatorMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
