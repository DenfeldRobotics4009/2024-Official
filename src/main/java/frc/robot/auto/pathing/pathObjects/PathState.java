// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.pathing.pathObjects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

// final struct format

public final class PathState {
    // The position and orientation the robot should attempt to drive to.
    public final Pose2d goalPose;
    // The speed in which the robot should attempt to drive to its goal point
    public final double speedMetersPerSecond;
    
    /**
     * Contains a single state within a robot path
     * @param posMeters
     * @param orientation
     * @param speedMetersPerSecond
     */
    public PathState (
        Translation2d gotoPosMeters,
        Rotation2d orientation,
        double speedMetersPerSecond
    ) {
        goalPose = new Pose2d(gotoPosMeters, orientation);
        this.speedMetersPerSecond = speedMetersPerSecond;
    }
}
