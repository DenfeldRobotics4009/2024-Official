// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.pathing.controllers;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * An interface that is capable of outputting both translation
 * and rotation speeds to the drivetrain.
 */
public interface PoseController {

    /**
     * @return Pose2d representing speeds to apply to the drive train.
     * These speeds will not be converted to field oriented after being
     * returned.
     */
    public Pose2d getPoseSpeeds();

}
