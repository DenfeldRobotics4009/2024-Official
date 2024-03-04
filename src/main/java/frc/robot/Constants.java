// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static double laserSensorVoltageHigh = 0.58;

    public static double inchesInMeter = 39.3700787402;

    public static final class Climber {
        public static int leftClimberMotorID = 18;
        public static int rightClimberMotorID = 19;
        public static double down = -5;
        public static double up = 450;
        public static double climberMotorPower = 1;  
    }
    public static final class Shooter {
        public static int topMotorID = 57;
        public static int bottomMotorID = 58; 
        public static double spin = 0.9;
        public static int feederMotorID = 59;
        public static double feederSpeed = 1;
        public static int aimMotorID = 40;
        
        public static double pidTolerance = 2;

        public static double aimRangeFrom0 = -133;
        public static double transferAngle = -90;
        
        public static double minimumFlywheelSpeed = 0;
        public static double maxShootSpeed = 1000;

        public static double flyWheelP = 0.00045;
        public static double flyWheelI = 0;
        public static double flyWheelD = 0;
        public static double flyWheelF = 0.00015;

        public static double flyWheelTolerance = 400; // RPM

        public static int barrelLaserSensorID = 0;

        public static int aimLimitSwitchID = 0;

        public static double flyWheelSpeed = 5440; // RPM

        public static double topAmpFlyWheelSpeed = 1000; // RPM
        public static double bottomAmpFlyWheelSpeed = 1671; // RPM
    }
    public static final class Intake {
        public static int intakeMotorID = 12;
        public static int rotateMotorID = 10; 
        public static double intakeMotorPower = -0.4;

        public static double ground = -0.3242;
        public static double deposit = -0.1315;

        public static double pidTolerance = 0.12;

        public static int intakeLaserSensorID = 1;
        public static int intakeInnerLimitSwitchID = 1;
        public static int intakeOuterLimitSwitchID = 2;

        public static double rotateEncoderOffset = -0.032;// 0.9709;
    }
    public static final class AprilTagOdometry {
        public static double maxSpeed = 0.001; // power
        public static double maxRotation = 0.001; // power
        public static Transform3d cameraPose = new Transform3d(
            new Translation3d(0, 0, 0.694), // TODO convert to meters
            new Rotation3d(0, 0, 0)
        );
        public static double yawToSpeakerOffset = -5;
    }

    public static final class Swerve {

        public static final class FrontLeft {
            public static int driveID = 21;
            public static int steerID = 31;
            public static int CANCoderID = 1;
            public static Rotation2d defaultCalibration = 
                new Rotation2d(Math.toRadians(0));
            public static Translation2d trackPosition =
                new Translation2d(-Swerve.TrackXMeters/2, -Swerve.TrackYMeters/2);
        }

        public static final class FrontRight {
            public static int driveID = 23;
            public static int steerID = 33;
            public static int CANCoderID = 2;
            public static Rotation2d defaultCalibration = 
                new Rotation2d(Math.toRadians(0));
            public static Translation2d trackPosition =
                new Translation2d(-Swerve.TrackXMeters/2, Swerve.TrackYMeters/2);
        }

        public static final class BackLeft {
            public static int driveID = 22;
            public static int steerID = 32;
            public static int CANCoderID = 3;
            public static Rotation2d defaultCalibration = 
                new Rotation2d(Math.toRadians(0));
            public static Translation2d trackPosition =
                new Translation2d(Swerve.TrackXMeters/2, -Swerve.TrackYMeters/2);
        }

        public static final class BackRight {
            public static int driveID = 24;
            public static int steerID = 34;
            public static int CANCoderID = 0;
            public static Rotation2d defaultCalibration = 
                new Rotation2d(Math.toRadians(0));
            public static Translation2d trackPosition =
                new Translation2d(Swerve.TrackXMeters/2, Swerve.TrackYMeters/2);
        }

        // Meters from wheel center to wheel center
        public static double TrackYMeters = 0.59325;
        public static double TrackXMeters = 0.59325;

        public static Rotation2d forwardAngle = new Rotation2d(Math.toRadians(0));
    }
        public static final class Paths {
            public static Translation2d START_LEFT = new Translation2d(0.24,6.51);
            public static Rotation2d START_LEFT_ANGLE = new Rotation2d(Math.toRadians(238));
            public static Translation2d START_CENTER = new Translation2d(0.8701, 5.528);
            public static Rotation2d START_CENTER_ANGLE = new Rotation2d(Math.toRadians(180));
            public static Translation2d START_RIGHT = new Translation2d(0.3,4.7);
            public static Rotation2d START_RIGHT_ANGLE = new Rotation2d(Math.toRadians(118.68));            
            public static Translation2d START_FAR_RIGHT = new Translation2d(0,1.622);
        }
}
