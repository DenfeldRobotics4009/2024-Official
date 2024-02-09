// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.pathing;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class PathingConstants {

    public static double endpointTolerance = 0.2;
    /**
     * Sets the default allowed distance from the robot to the
     * last point in the path for the path command to end.
     * @param tolerance
     */
    public static void setDefaultEndpointTolerance(double tolerance) {
        PathingConstants.endpointTolerance = FollowPath.Clamp(tolerance, 1, 0);
    }
    
    public static Rotation2d forwardAngle = new Rotation2d();
    /**
     * Sets the angle offset paths will be adjusted by,
     * allowing what is considered the front of the robot
     * to be changed.
     * @param angle Rotation2d
     */
    public static void setForwardAngle(Rotation2d forwardAngle) {
        PathingConstants.forwardAngle = forwardAngle;
    }

    public static double lookAheadScalar = 1.5;
    /**
     * Sets the multiplier used to calculate lookahead
     * from the current speed of the robot along a path.
     * lookAheadMeters = speedMeters * lookAheadScalar
     * @param lookAheadScalar double, [0.1, 10]
     */
    public static void setLookAheadScalar(double lookAheadScalar) {
        PathingConstants.lookAheadScalar = 
            FollowPath.Clamp(lookAheadScalar, 10, 0.1);
    }

    public static double maxVelocityMeters = 5.06; // Mk4I Swerve Module
    /**
     * Sets the maximum allowed speed of paths
     * @param maxVelocityMeters double [0, 10]
     */
    public static void setMaxVelocityMeters(double maxVelocityMeters) {
        PathingConstants.maxVelocityMeters = 
            FollowPath.Clamp(maxVelocityMeters, 10, 0);
    }

    public static double maxAccelerationMeters = 3;
    /**
     * Sets the maximum allowed decceleration along paths
     * This does not limit acceleration, only decelleration.
     * @param maxAccelerationMeters double [0, 10]
     */
    public static void setMaxAccelerationMeters(double maxAccelerationMeters) {
        PathingConstants.maxAccelerationMeters = 
            FollowPath.Clamp(maxAccelerationMeters, 10, 0);
    }

    public static double turningProportion = 5;
    /**
     * Sets the proportion of the distance from the goal orientation
     * to use when rotating the robot toward the goal orientation
     */
    public static void setTurningProportion(double turningProportion) {
        PathingConstants.turningProportion = turningProportion;
    }

    public static DriveSubsystem driveSubsystem = null; // Will not run unless set
    /**
     * Sets the drive train subsystem object that will be used
     * by the pathing algorithm. If this is not set, a
     * the code will crash when it attempts to run. This should be
     * the last constant to be set during initialization. 
     * @param driveSubsystem Interface which describes a generic
     * swerve drive subsystem.
     */
    public static void setDriveSubsystem(DriveSubsystem driveSubsystem) {
        PathingConstants.driveSubsystem = driveSubsystem;
    }
}
