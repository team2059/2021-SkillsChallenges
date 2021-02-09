package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class AlignToBall extends PIDCommand {

    public AlignToBall(DriveBase driveBase){
        super(
                new PIDController(Constants.DriveConstants.PID.kTurnP, Constants.DriveConstants.PID.kTurnI, Constants.DriveConstants.PID.kTurnD),
                driveBase::getBallOffset,
                0,
                output -> driveBase.arcadeDrive(-(output + (1 * Math.signum(output))),  0),
                driveBase
        );
        getController().setTolerance(.5, 0);
    }

    @Override
    public void initialize() {
        System.out.println("Starting auto align");
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint(); //|| NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0) == 0;
    }
}
