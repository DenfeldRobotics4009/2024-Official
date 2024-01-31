// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.odometry;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public interface OdometrySource {
    /**
     * @return Pose2d of robot pose if present
     */
    Optional<Pose2d> getPosition();
    /**
     * External odometry sources will often
     * nullify this value, and cannot be set.
     * @param position Pose2d
     */
    void setPosition(Pose2d position);
}
