// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Field {

    /**
     * X distance from blue zero to red zero
     */
    public static final double fieldLengthMeters = 16.54175;

    /**
     * Y distance from red zero to blue zero
     */
    public static final double fieldWidthMeters = 8.22325;

    /**
     * Translates a position relative to the blue 
     * alliance zero to a position relative to a red
     * alliance zero.
     * @param bluePosition
     * @return
     */
    public static Pose2d translateRobotPoseToRed(Pose2d bluePosition) {
        return new Pose2d(
            new Translation2d(fieldLengthMeters - bluePosition.getX(), fieldWidthMeters - bluePosition.getY()),
            bluePosition.getRotation().plus(new Rotation2d(Math.PI))
        );
    }

    public static Pose2d flipPoint(Pose2d point) {
        return new Pose2d(
            new Translation2d(point.getX(), fieldWidthMeters - point.getY()),
            point.getRotation().plus(Rotation2d.fromRadians(Math.PI))
        );
    }

    /**
     * Translates a position relative to the blue 
     * alliance zero to a position relative to a red
     * alliance zero.
     * @param bluePosition
     * @return
     */
    public static Translation2d translateRobotPoseToRed(Translation2d bluePosition) {
        return new Translation2d(fieldLengthMeters - bluePosition.getX(), fieldWidthMeters - bluePosition.getY());
    }

    /**
     * @return true if the robot is on the red alliance
     */
    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    }

    public static Pose2d flipPointIfRed(Pose2d point) {
        if (isRedAlliance()) {
            return flipPoint(point);
        }
        
        return point;
    }
}
