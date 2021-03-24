package frc.robot.commands.AutoHelpers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

public class GoToThorPos extends CommandBase {
    private final DriveBase drivebase;
    private int target;

    public GoToThorPos(DriveBase subsystem, int thor) {
        this.drivebase = subsystem;
        this.target = thor;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Starting Drive");
    }

    @Override
    public void execute() {
        drivebase.arcadeDrive(-.1, -.4);
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.tankDriveVolts(0, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(RobotContainer.limelight.getEntry("thor").getDouble(0.0) - this.target) < 5;
    }
}