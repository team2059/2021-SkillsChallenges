/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
        public static final int leftPrimarySlave = 11;
        public static final int leftSecondarySlave = 12;
        public static final int leftMaster = 13;
        public static final int rightMaster = 14;
        public static final int rightPrimarySlave = 15;
        public static final int rightSecondarySlave = 16;

        public static final double kGearRatio = (double) 625 / 49;
        public static final double kWheelRadiusInches = 3.0;

        public final class PID {
            public static final double kP = 0.05;
            public static final double kI = 0.0;
            public static final double kD = 0.006;

            public static final double kTurnP = 6;
            public static final double kTurnI = 0.0;
            public static final double kTurnD = 0;
        }
    }

    public static final class TurretConstants {
        public static final int TurretRotatorTalonPort = 20;
        public static final int LeftLimitHallEffectPort = 0;
        public static final int RightLimitHallEffectPort = 2;

        public static final int LeftLimit = -2170;
        public static final int RightLimit = 35165;

        public static final double RotatekP = .075;
        public static final double RotatekI = 0.0;
        public static final double RotatekD = 0;

        public static final double EncoderRotatekP = .001;
        public static final double EncoderRotatekI = 0.0;
        public static final double EncoderRotatekD = 0.0;

        public static final double offsetTolerance = .5;
        public static final double offsetVelocityTolerance = 0;
    }


    public static final class ClimberConstants {
        public static final int ArmLifterMotorPort = 19;
        public static final int WinchMotorPort = 22;
        public static final int TrunionMotorPort = 25;

        public static final int LoopIdx = 0;
        public static final int MotorSlotIdx = 0;

        public static final int CtreTimeoutMs = 30;

        public static final double ArmKf = 0;
        public static final double ArmKp = .1;
        public static final double ArmkI = 0;
        public static final double ArmkD = 0;
    }

    public static final class ShooterConstants {
        public static final int ShooterFlywheelTalonPort = 24;
        public static final int HoodMotorSparkMaxPort = 17;
        public static final int HoodPotPort = 0;

        public static final double FlyWheelkF = 0;
        public static final double FlyWheelkP = .025;
        public static final double FlyWheelkI = 0.00009;
        public static final double FlyWheelkD = 0.4;

        public static final int LoopIdx = 0;
        public static final int MotorSlotIdx = 0;

        public static final int CtreTimeoutMs = 30;

        // PID coefficients
        public static final double HoodkP = .1;
        public static final double HoodkI = .001;
        public static final double HoodkD = .001;
        public static final double HoodkIz = 0;
        public static final double HoodkFF = 0;
        public static final double HoodkMaxOutput = .5;
        public static final double HoodkMinOutput = -.5;

        // Hood encoded setpoints
        public static final double zeroHoodPosition = 0;
        public static final double lowGoalHoodPosition = 10;

        // Smart Motion Coefficients
        public static final double HoodMaxVel = 750; // rpm
        public static final double HoodMinVel = 0;
        public static final double HoodMaxAcc = 500;

        public static final double HoodAllowedErr = .05;
    }

    public static final class ConveyorConstants {
        // TODO Change collector and Trunion motor ports
        public static final int BallElevatorTalonPort = 21;
        public static final int CollectorMotor = 18;
    }

    public static final class FieldConstants {
        // distance to target is 50
        public static final double HighGoalHeight = 89.5;
        public static final double LimeLightHeight = 20;
        public static final double LimelightAngle = 22;

        public static boolean isTenFootShot = true;
        public static double shootLength = Constants.FieldConstants.isTenFootShot ? 3.85 : 4.5;
    }
}
