// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.pathing.controllers;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * An interface that is capable of outputting rotational 
 * speeds to the drivetrain.
 */
public interface RotationController {

    /**
     * @return Rotation2d representing rotational speeds to apply to the 
     * drive train.
     */
    public Rotation2d getRotationSpeeds();

}
