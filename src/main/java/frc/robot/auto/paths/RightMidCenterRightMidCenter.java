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
public class RightMidCenterRightMidCenter extends Path {
    /**
     * Drives the robot 1.5 meters along the x
     * axis (which should be forward, if the settings
     * are properly configured within the PathingConstants
     * class), at the robots maximum speed.
     */
    public RightMidCenterRightMidCenter() {
        /**
         * Initializes Path super with the given
         * set of points, the first point passed
         * into super is the first point along the path.
         */
        super(            
            new PathPoint(
                new Translation2d(0,1.622),               // Starting Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Started here")       // Command 
            ),
            new PathPoint(
                new Translation2d(3.243,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 1")       // Command 
            ),
            new PathPoint(
                new Translation2d(4.379,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 2")       // Command 
            ),
            new PathPoint(
                new Translation2d(5.433,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(-Math.PI/4)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 3")       // Command 
            ),
            new PathPoint(
                new Translation2d(6.162,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(-Math.PI/4)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 4")       // Command 
            ),
            new PathPoint(
                new Translation2d(6.729,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(-Math.PI/2)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 5")       // Command 
            ),
            new PathPoint(
                new Translation2d(7.541,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(-Math.PI/2)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 6")       // Command 
            ),
            new PathPoint(
                new Translation2d(8.189,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(-Math.PI*(3)/4)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 7")       // Command 
            ),
            new PathPoint(
                new Translation2d(8.676,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(-Math.PI*(3)/4)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 8")       // Command 
            ),
            new PathPoint(
                new Translation2d(8.676,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(-Math.PI*(3)/4)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 9")       // Command 
            ),
            new PathPoint(
                new Translation2d(8.108,2.433),               // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at MidCenterRight")       // Command 
            ),
            new PathPoint(
                new Translation2d(5.676,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at Shoot Path point 1")       // Command 
            ),
            new PathPoint(
                new Translation2d(4.865,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at Shoot Path point 2")       // Command 
            ),
            new PathPoint(
                new Translation2d(3.729,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at Shoot Path point 3")       // Command 
            ),
            new PathPoint(
                new Translation2d(1.784,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at Shoot Path point 4")       // Command 
            ),
            new PathPoint(
                new Translation2d(3.811,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 1")       // Command 
            ),
            new PathPoint(
                new Translation2d(5.352,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(-Math.PI/2)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 2")       // Command 
            ),
            new PathPoint(
                new Translation2d(6.487,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(-Math.PI*(3)/4)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 3")       // Command 
            ),
            new PathPoint(
                new Translation2d(7.459,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(-Math.PI*(3)/4)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 4")       // Command 
            ),
            new PathPoint(
                new Translation2d(8.027,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 5")       // Command 
            ),
            new PathPoint(
                new Translation2d(8.595,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 6")       // Command 
            ),
            new PathPoint(
                new Translation2d(8.676,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 7")       // Command 
            ),
            new PathPoint(
                new Translation2d(8.676,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 8")       // Command 
            ),
            new PathPoint(
                new Translation2d(8.108,3.973),               // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI)),     // Rotation (rad)
            1,    // Speed (m/s)
                new PrintCommand("Arrived at MidCenter")       // Command
            ),
            new PathPoint(
                new Translation2d(5.676,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at Shoot Path point 1")       // Command 
            ),
            new PathPoint(
                new Translation2d(4.865,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at Shoot Path point 2")       // Command 
            ),
            new PathPoint(
                new Translation2d(3.729,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at Shoot Path point 3")       // Command 
            ),
            new PathPoint(
                new Translation2d(1.784,0),                       // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at Shoot Path point 4")       // Command 
            )
        );
    }

}
