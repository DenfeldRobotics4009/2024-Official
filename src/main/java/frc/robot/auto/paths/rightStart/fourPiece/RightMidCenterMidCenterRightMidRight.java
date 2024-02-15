// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.paths.rightStart.fourPiece;

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
public class RightMidCenterMidCenterRightMidRight extends Path {
    /**
     * Drives the robot 1.5 meters along the x
     * axis (which should be forward, if the settings
     * are properly configured within the PathingConstants
     * class), at the robots maximum speed.
     */
    public RightMidCenterMidCenterRightMidRight() {
        /**
         * Initializes Path super with the given
         * set of points, the first point passed
         * into super is the first point along the path.
         */
        super(            
            new PathPoint(
                Constants.Paths.START_RIGHT,               // Starting Position (meters)
                new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Started in Starting Position")       // Command 
            ),
            new PathPoint(
                new Translation2d(1.946,2.027),               // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI/2)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Shoot")       // Command 
            ),
            new PathPoint(
                new Translation2d(3,2.433),               // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI/2)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 1")       // Command 
            ),
            new PathPoint(
                new Translation2d(3.649,2.919),               // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI/4)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 2")       // Command 
            ),
            new PathPoint(
                new Translation2d(4.135,3.324),               // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI/4)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 3")       // Command 
            ),
            new PathPoint(
                new Translation2d(4.865,3.892),               // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI/4)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 4")       // Command 
            ),
            new PathPoint(
                new Translation2d(5.676,4.135),               // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI/4)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 5")       // Command 
            ),
            new PathPoint(
                new Translation2d(6.487,4.297),               // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI/4)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 6")       // Command 
            ),
            new PathPoint(
                new Translation2d(7.298,4.459),               // Position (meters)
                new Rotation2d(Math.toRadians(Math.PI/4)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 7")       // Command 
            ),
            new PathPoint(
                new Translation2d(8.189,4.216),               // Position (meters)
                new Rotation2d(Math.toRadians(-Math.PI/2)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Intake")       // Command 
            ),
            new PathPoint(
                new Translation2d(7.946,3.729),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 8")       // Command 
            ),
            new PathPoint(
                new Translation2d(7.379,3.649),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 9")       // Command 
            ),
            new PathPoint(
                new Translation2d(6.487,3.729),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 10")       // Command 
            ),
            new PathPoint(
                new Translation2d(5.838,3.649),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 11")       // Command 
            ),
            new PathPoint(
                new Translation2d(5.189,3.487),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 12")       // Command 
            ),
            new PathPoint(
                new Translation2d(4.703,3.243),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 13")       // Command 
            ),
            new PathPoint(
                new Translation2d(4.379,2.838),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 1 point 14")       // Command 
            ),
            new PathPoint(
                new Translation2d(3.973,2.676),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Shoot")       // Command 
            ),
            new PathPoint(
                new Translation2d(4.626,2.433),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 1")       // Command 
            ),
            new PathPoint(
                new Translation2d(4.541,2.189),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 2")       // Command 
            ),
            new PathPoint(
                new Translation2d(5.027,2.108),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 3")       // Command 
            ),
            new PathPoint(
                new Translation2d(5.433,2.108),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 4")       // Command 
            ),
            new PathPoint(
                new Translation2d(5.433,2.108),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 5")       // Command 
            ),
            new PathPoint(
                new Translation2d(6.487,2.189),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 6")       // Command 
            ),
            new PathPoint(
                new Translation2d(7.135,2.351),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 7")       // Command 
            ),
            new PathPoint(
                new Translation2d(8.027,2.433),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 8")       // Command 
            ),
            new PathPoint(
                new Translation2d(8.514,2.595),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Intake")       // Command 
            ),
            new PathPoint(
                new Translation2d(8.189,2.189),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 9")       // Command 
            ),
            new PathPoint(
                new Translation2d(7.541,1.946),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 10")       // Command 
            ),
            new PathPoint(
                new Translation2d(7.054,1.701),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 11")       // Command 
            ),
            new PathPoint(
                new Translation2d(6.487,1.622),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 12")       // Command 
            ),
            new PathPoint(
                new Translation2d(5.919,1.622),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 13")       // Command 
            ),
            new PathPoint(
                new Translation2d(5.352,1.622),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 14")       // Command 
            ),
            new PathPoint(
                new Translation2d(4.946,1.784),               // Position (meters)
                new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
                1,    // Speed (m/s)
                new PrintCommand("Arrived at arc 2 point 15")       // Command 
            )

        );
    }

}
