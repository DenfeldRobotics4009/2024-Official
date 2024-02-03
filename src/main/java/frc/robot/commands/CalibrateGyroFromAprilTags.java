// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.odometry.AprilTagOdometry;
import frc.robot.subsystems.SwerveDrive;

public class CalibrateGyroFromAprilTags extends Command {
  
  AprilTagOdometry camera;

  public CalibrateGyroFromAprilTags(AprilTagOdometry camera) {
    this.camera = camera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<Pose2d> pose = camera.getPosition();
    if (pose.isPresent()) {
      SwerveDrive.inverseKinematics.setRotation(pose.get().getRotation());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
