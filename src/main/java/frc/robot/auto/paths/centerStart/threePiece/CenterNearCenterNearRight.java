// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.paths.centerStart.threePiece;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;

/**
 * Example pure pursuit path, showing a basic
 * and readable implementation.
 */
public class CenterNearCenterNearRight extends Path {
    /**
     * Drives the robot 1.5 meters along the x
     * axis (which should be forward, if the settings
     * are properly configured within the PathingConstants
     * class), at the robots maximum speed.
     */
    public CenterNearCenterNearRight() {
        /**
         * Initializes Path super with the given
         * set of points, the first point passed
         * into super is the first point along the path.
         */
        super(            
            0.5, // End tolerance
            new PathPoint(
                Constants.Paths.START_CENTER,               // Position (meters)
                new Rotation2d(Math.toRadians(180)),     // Rotation (rad)
                3,    // Speed (m/s)
                new PrintCommand("Shoot")       // Command 
            ),
            new PathPoint(
                new Translation2d(1.5,0),               // Position (meters)
                new Rotation2d(Math.toRadians(180)),     // Rotation (rad)
                2,    // Speed (m/s)
                new PrintCommand("Intake")       // Command 
            ),
            new PathPoint(
                new Translation2d(0.5,-1),               // Position (meters)
                new Rotation2d(Math.toRadians(180)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Transfer to next")       // Command 
            ),
            new PathPoint(
                new Translation2d(1.5,-2),               // Position (meters)
                new Rotation2d(Math.toRadians(180)),     // Rotation (rad)
                0,    // Speed (m/s)
                new PrintCommand("Intake")       // Command 
            )
        );
    }

}
