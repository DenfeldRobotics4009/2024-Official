// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.paths;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;

/**
 * Example pure pursuit path, showing a basic
 * and readable implementation.
 */
public class ExamplePath extends Path {
    /**
     * Drives the robot 1.5 meters along the x
     * axis (which should be forward, if the settings
     * are properly configered within the PathingConstants
     * class), at the robots maximum speed.
     */
    public ExamplePath() {
        /**
         * Initializes Path super with the given
         * set of points, the first point passed
         * into super is the first point along the path.
         */
        super(
            new PathPoint(
                new Translation2d(0, 0),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                2,    // Speed (m/s)
                new PrintCommand("Past point 0")       // Command 
            ),
            new PathPoint(
                new Translation2d(1.5, 0),
                new Rotation2d(Math.toRadians(180)), 
                0, 
                new PrintCommand("Past point 1")
            )
        );
    }

}
