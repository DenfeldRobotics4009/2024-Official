// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

    public static final class AprilTagOdometry {
        public static int sampleSize = 8;
        public static Transform2d maximumDeviation = new Transform2d(
            new Translation2d(0.1, 0.1), new Rotation2d(Math.PI/12)
        );
        public static double maxSampleAge = 100;
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
}
