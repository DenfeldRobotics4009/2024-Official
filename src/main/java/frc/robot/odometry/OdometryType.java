// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.odometry;

public enum OdometryType {
    /**
     * External odometry sources are non-continuous, non-self relative sources
     * for position calculation. Thus, they do not have to constantly supply
     * an accurate and up to date Pose2d when getPosition is called, instead
     * returning an empty optional type. These position sources are assumed
     * to be more accurate than internal sources due to external sources
     * having non-compounding error.
     */
    External, 
    
    /**
     * Internal odometry sources provide continuous, self-relative position
     * calculations. Thus, they (in most cases) can constantly provide an
     * accurate and up to date Pose2d when getPosition is called. These position
     * sources are assumed to be less accurate than external sources due
     * to error compounding over time.
     */
    Internal;
}
