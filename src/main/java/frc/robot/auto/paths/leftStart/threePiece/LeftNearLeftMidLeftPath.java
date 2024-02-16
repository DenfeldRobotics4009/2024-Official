// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.paths.leftStart.threePiece;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;

/**
 * Example pure pursuit path, showing a basic
 * and readable implementation.
 */
public class LeftNearLeftMidLeftPath extends Path {
    /**
     * Drives the robot 1.5 meters along the x
     * axis (which should be forward, if the settings
     * are properly configured within the PathingConstants
     * class), at the robots maximum speed.
     */
    public LeftNearLeftMidLeftPath() {
        /**
         * Initializes Path super with the given
         * set of points, the first point passed
         * into super is the first point along the path.
         */
        super(            
            new PathPoint(
                new Translation2d(0,0),               // Starting Position
                new Rotation2d(Math.toRadians(239)),     // Rotation (rad)
                4,    // Speed (m/s)
                new PrintCommand("Started here")       // Command 
            ),
            new PathPoint(
                new Translation2d(0.5,0),               // Starting Position
                new Rotation2d(Math.toRadians(239)),     // Rotation (rad)
                4,    // Speed (m/s)
                new PrintCommand("Beginning rotation to 270")       // Command 
            ),
            new PathPoint(
                new Translation2d(2,-0.5),               // Starting Position
                new Rotation2d(Math.toRadians(270)),     // Rotation (rad)
                2,    // Speed (m/s)
                new PrintCommand("Moving in to grab")       // Command 
            ),
            new PathPoint(
                new Translation2d(2.5,0.5),               // Starting Position
                new Rotation2d(Math.toRadians(270)),     // Rotation (rad)
                2, // Speed (m/s)
                new PrintCommand("Initiating transfer and shoot")       // Command 
            ),
            new PathPoint(
                new Translation2d(3,-0.5),               // Starting Position
                new Rotation2d(Math.toRadians(180)),     // Rotation (rad)
                2,    // Speed (m/s)
                new PrintCommand("Shoot")       // Command 
            ),
            new PathPoint(
                new Translation2d(8.5,-0.4),               // Starting Position
                new Rotation2d(Math.toRadians(270)),     // Rotation (rad)
                2,    // Speed (m/s)
                new PrintCommand("Ending")       // Command 
            ),
            new PathPoint(
                new Translation2d(8.5,0),               // Starting Position
                new Rotation2d(Math.toRadians(270)),     // Rotation (rad)
                2,    // Speed (m/s)
                new PrintCommand("Ending")       // Command 
            ),
            new PathPoint(
                new Translation2d(8.5,0.1),               // Starting Position
                new Rotation2d(Math.toRadians(270)),     // Rotation (rad)
                4,    // Speed (m/s)
                new PrintCommand("Ending")       // Command 
            ),
            new PathPoint(
                new Translation2d(7,0.1),               // Starting Position
                new Rotation2d(Math.toRadians(270)),     // Rotation (rad)
                4,    // Speed (m/s)
                new PrintCommand("Ending")       // Command 
            ),
            new PathPoint(
                new Translation2d(3,-0.5),               // Starting Position
                new Rotation2d(Math.toRadians(180)),     // Rotation (rad)
                0,    // Speed (m/s)
                new PrintCommand("Ending")       // Command 
            )

            // new PathPoint(
            //     new Translation2d(0,7.054),               // Starting Position
            //     new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
            //     1,    // Speed (m/s)
            //     new PrintCommand("Started here")       // Command 
            // ),
            // new PathPoint(
            //     new Translation2d(2.838,7.054),               // Position (meters)
            //     new Rotation2d(Math.toRadians(0)),     // Rotation (rad)
            //     1,    // Speed (m/s)
            //     new PrintCommand("Arrived at NearLeft")       // Command 
            // ),
            // new PathPoint(
            //     new Translation2d(7.298,6.729),               // Position (meters)
            //     new Rotation2d(Math.toRadians(0)), //-Math.PI*(3)/4)),     // Rotation (rad)
            //     1,    // Speed (m/s)
            //     new PrintCommand("Arrived at arc 1 point 1")       // Command 
            // ),
            // new PathPoint(
            //     new Translation2d(8.108,6.649),               // Position (meters)
            //     new Rotation2d(Math.toRadians(0)), //-Math.PI*(3)/4)),     // Rotation (rad) (may need to add rotation here depending on how fast our robot is spinning)
            //     1,    // Speed (m/s)
            //     new PrintCommand("Arrived at arc 1 point 2")       // Command 
            // ),
            // new PathPoint(
            //     new Translation2d(8.514,6.811),               // Position (meters)
            //     new Rotation2d(Math.toRadians(0)), //-Math.PI*(3)/4)),     // Rotation (rad) (may need to add rotation here depending on how fast our robot is spinning)
            //     1,    // Speed (m/s)
            //     new PrintCommand("Arrived at arc 1 point 3")       // Command 
            // ),
            // new PathPoint(
            //     new Translation2d(8.838,7.054),               // Position (meters)
            //     new Rotation2d(Math.toRadians(0)), //-Math.PI*(3)/4)),     // Rotation (rad) (may need to add rotation here depending on how fast our robot is spinning)
            //     1,    // Speed (m/s)
            //     new PrintCommand("Arrived at arc 1 point 4")       // Command 
            // ),
            // new PathPoint(
            //     new Translation2d(8.879,7.298),               // Position (meters)
            //     new Rotation2d(Math.toRadians(0)), //-Math.PI*(3)/4)),     // Rotation (rad) (may need to add rotation here depending on how fast our robot is spinning)
            //     1,    // Speed (m/s)
            //     new PrintCommand("Arrived at arc 1 point 5")       // Command 
            // ),                          
            // new PathPoint(
            //     new Translation2d(8.757,7.459),               // Position (meters)
            //     new Rotation2d(Math.toRadians(0)), //-Math.PI*(3)/4)),     // Rotation (rad) (may need to add rotation here depending on how fast our robot is spinning)
            //     1,    // Speed (m/s)
            //     new PrintCommand("Arrived at arc 1 point 6")       // Command 
            // ),       
            // new PathPoint(
            //     new Translation2d(8.352,7.459),               // Position (meters)
            //     new Rotation2d(Math.toRadians(0)), //Math.PI)),     // Rotation (rad)
            //     1,    // Speed (m/s)
            //     new PrintCommand("Arrived at MidLeft")       // Command 
            // ),
            // new PathPoint(
            //     new Translation2d(4.622,7.459),               // Position (meters)
            //     new Rotation2d(Math.toRadians(Math.PI)),     // Rotation (rad)
            //     1,    // Speed (m/s)
            //     new PrintCommand("Arrived at shooting point")       // Command 
            // )       
            );
    }

}
