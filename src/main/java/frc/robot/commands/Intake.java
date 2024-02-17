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
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;

public class Intake extends Command implements AutoRotationSource{
  IntakeSubsystem intake;
  NoteCamera camera;
  /**
   * Runs the intake until the sensor is activated
   * @param intake
   */
  public Intake(IntakeSubsystem intake, NoteCamera camera) {
    addRequirements(intake);
    this.intake = intake;
    this.camera = camera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getIntakeSensor();
  }

  @Override
  public Optional<Rotation2d> getGoalRotation() {
    Optional<Double> yawToNote = camera.getYawToNote();
    if (yawToNote.isPresent()) {
      return Optional.of(
        new Rotation2d()
        // // Current yaw to note, plus current drive angle
        // new Rotation2d(Math.toRadians(yawToNote.get())).plus(
        //   SwerveDrive.getInstance().getPosition().getRotation()
        // )
      );
    }

    return Optional.empty();
  }
}
