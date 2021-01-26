/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveBase;
import frc.robot.Constants.drivePID;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoDrive extends PIDCommand {
  /**
   * Creates a new AutoDrive.
   */
  public AutoDrive(DriveBase db, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(drivePID.kP, drivePID.kI, drivePID.kD),
        // This should return the measurement
        db::getDistance,
        // This should return the setpoint (can also be a constant)
        setpoint,
        // This uses the output
        output -> {
          db.tankDrive(output, -output);
        });

        getController().setTolerance(20);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void initialize() {
    System.out.println("Staring PID Straight Drive");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
