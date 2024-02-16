// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.pathing;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;

/**
 * A command to implement external source rotation
 */
public interface AutoShoot {
    
    /**
     * Gets the goal rotation for the path
     * @return Rotation2d of goal angle
     */
    public Rotation2d getGoalRotation();
}
