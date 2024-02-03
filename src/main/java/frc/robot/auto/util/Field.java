// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.util;

import edu.wpi.first.math.geometry.Translation2d;

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
    public static Translation2d translateRobotPoseToRed(Translation2d bluePosition) {
        return new Translation2d(
            fieldLengthMeters - bluePosition.getX(), 
            fieldWidthMeters - bluePosition.getY()
        );
    }
}
