package frc.robot.commands.PIDDrive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class AutoRotate extends PIDCommand {
    public AutoRotate(DriveBase driveBase, double setpoint){
        super(
                new PIDController(Constants.DriveConstants.PID.kP, Constants.DriveConstants.PID.kI, Constants.DriveConstants.PID.kD),
                driveBase::getGyroAngle,
                setpoint,
                output -> driveBase.arcadeDrive(0, output),
                driveBase
        );

        getController().setTolerance(15, 0);
    }

    @Override
    public void initialize() {
        System.out.println("Starting auto straight drive");
    }
}
