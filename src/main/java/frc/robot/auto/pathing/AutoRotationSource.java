// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.pathing;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A command to implement external source rotation
 */
public interface AutoRotationSource {
    
    /**
     * Gets the goal rotation for the path
     * @return Rotation2d of goal angle
     */
    public Optional<Rotation2d> getGoalRotation();
}

