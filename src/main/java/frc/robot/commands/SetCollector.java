package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Turret;

public class SetCollector extends CommandBase {
    private final Conveyor m_Conveyor;
    private double speed;

    public SetCollector(Conveyor subsystem, double speed) {
        this.m_Conveyor = subsystem;
        this.speed = speed;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Starting Turret");
    }

    @Override
    public void execute() {
//        RobotContainer.getClimber().setTrunionMotor(speed); // TODO: REPLACE THE TRUNION MOTOR SO THE COLLECTOR WORKS ON THE RIGHT PORT
        m_Conveyor.setCollectorMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.getClimber().setTrunionMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
