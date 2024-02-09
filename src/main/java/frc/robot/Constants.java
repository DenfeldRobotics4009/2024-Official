// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Arm {
        public static int armIntakeMotorID = 63;
        public static int armShooterMotorLeftID = 64;
        public static int armShooterMotorRightID = 65;
        public static int armMotorID = 80;
        public static double armIntakeMotorPower = 0.4;
        public static double source = 1000;
        public static double ground = 500;
        public static double deposit = 250;
        public static double upperLimit = 6.25;
        public static double lowerLimit = 0;
    }
    public static final class Climber {
        public static int climberMotorID = 70;
        public static double down = 1000;
        public static double up = 500;
        public static double climberMotorPower = 0.5;  
    }
    public static final class Turret {
        public static int topMotorID = 57;
        public static int bottomMotorID = 58; 
        public static double spin = 0.9;
        public static int feederMotorID = 59;
        public static double feederSpeed = 20;
        public static int aimMotorID = 40;
        public static double aimTolerance = 0.01;
        public static double minimumFlywheelSpeed = 0;
        public static double maxShootSpeed = 1000;

        public static double flyWheelP = 0.00045;
        public static double flyWheelI = 0;
        public static double flyWheelD = 0;
        public static double flyWheelF = 0.00015;

        public static double flyWheelTolerance = 400; // RPM
    }
    public static final class Intake {
        public static int intakeMotorID = 40;
        public static int rotateMotorID = 41; 
        public static double rotateMotorPower = 0.5;
        public static double intakeMotorPower = 0.5;
        public static double source = 1000;
        public static double ground = 500;
        public static double deposit = 250;
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
        public static double TrackYMeters = 0.65405;
        public static double TrackXMeters = 0.57785;

        public static Rotation2d forwardAngle = new Rotation2d(Math.toRadians(0));
    }
}
