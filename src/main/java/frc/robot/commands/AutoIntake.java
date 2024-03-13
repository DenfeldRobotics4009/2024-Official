// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.pathing.AutoRotationSource;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteCamera;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

public class AutoIntake extends TransferIntake implements AutoRotationSource {

  /**
   * Runs the intake until the sensor is activated
   * @param intake
   */
  public AutoIntake(
    IntakeSubsystem intake, 
    NoteCamera camera,
    Shooter shooter 
  ) {
    super(intake, shooter, camera);
  }

  @Override
  public Optional<Rotation2d> getGoalRotation() {
    Optional<Double> yawToNote = camera.getYawToNote();
    if (yawToNote.isPresent()) {
      return Optional.of(
        new Rotation2d(Math.toRadians(yawToNote.get() / 2))
      );
    }

    return Optional.empty();
  }
}
