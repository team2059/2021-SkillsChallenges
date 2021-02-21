package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveBase;

public class Drive extends CommandBase {
    private final DriveBase drivebase;

    public Drive(DriveBase subsystem) {
        this.drivebase = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Starting Drive");
    }

    @Override
    public void execute() {
        drivebase.drive();
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.tankDriveVolts(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}