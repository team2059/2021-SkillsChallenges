/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int leftForwardMotorPort = 11;
    public static final int leftMiddleMotorPort = 12;
    public static final int leftBackMotorPort = 13;
    public static final int rightForwardMotorPort = 14;
    public static final int rightMiddleMotorPort = 15;
    public static final int rightBackMotorPort = 16;

    public final class drivePID {
        public static final double kP = 0.05;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 1;
        public static final int kLeftMotor2Port = 2;
        public static final int kRightMotor1Port = 3;
        public static final int kRightMotor2Port = 4;
    
        public static final double kTrackwidthMeters = .53;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kEncoderCPR = 4096;
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kLinearDistancePerMotorRotation = 1 / 10.64;
        public static final double fudgeFactor = 1.0499;
            // // Assumes the encoders are directly mounted on the wheel shafts
            // (kWheelDiameterMeters * Math.PI * 12.57) / kEncoderCPR;
    
        public static final double ksVolts = 0.216;
        public static final double kvVoltSecondsPerMeter = 1.53;
        public static final double kaVoltSecondsSquaredPerMeter = 0.272;
    
        public static final double kPDriveVel = 1;

      }
    
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = .25;
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
