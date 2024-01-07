// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.pathing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A generic drive subsystem format utilized by
 * the autonomous pathing algorithm, and implemented
 * by the user.
 */
public interface DriveSubsystem extends Subsystem {
    /**
     * Drives the robot at the given field
     * oriented direction
     * @param speeds units in meters/sec and rad/sec
     */
    void drive(ChassisSpeeds speeds);

    /**
     * @return the current most accurate field
     * oriented position of the robot in meters.
     */
    Pose2d getPosition();
}
