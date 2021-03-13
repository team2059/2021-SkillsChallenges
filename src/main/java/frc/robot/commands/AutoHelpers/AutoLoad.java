package frc.robot.commands.AutoHelpers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallElevator;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Turret;

import java.util.Set;

public class AutoLoad extends CommandBase {
    private final BallElevator m_BallElevator;
    private double speed;

    public AutoLoad(BallElevator subsystem, double speed) {
        this.m_BallElevator = subsystem;
        this.speed = speed;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Loading Next Ball \n\n\n");
    }

    @Override
    public void execute() {
        if (Math.abs(RobotContainer.getShooter().getFlyWheelVelocity()) > 18950){
            SmartDashboard.putBoolean("Loading Next", true);
            m_BallElevator.setBallElevatorMotor(speed);
        } else {
            SmartDashboard.putBoolean("Loading Next", false);
            m_BallElevator.setBallElevatorMotor(0);
        }
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
