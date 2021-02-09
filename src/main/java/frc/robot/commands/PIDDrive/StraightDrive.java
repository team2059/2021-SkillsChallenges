package frc.robot.commands.PIDDrive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class StraightDrive extends PIDCommand {
    public StraightDrive(DriveBase driveBase, double setpoint){
        super(
                new PIDController(Constants.DriveConstants.PID.kP, Constants.DriveConstants.PID.kI, Constants.DriveConstants.PID.kD),
                driveBase::getStraightDistance,
                setpoint,
                output -> driveBase.tankDrive(output, output),
                driveBase
        );

        getController().setTolerance(15, 0);
    }

    @Override
    public void initialize() {
        System.out.println("Starting auto straight drive");
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
