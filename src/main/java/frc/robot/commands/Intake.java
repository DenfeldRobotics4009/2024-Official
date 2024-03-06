// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.pathing.controllers.RotationController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteCamera;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;

public class Intake extends Command implements RotationController {
  IntakeSubsystem intake;
  NoteCamera camera;

  Rotation2d goalOrientation;
  PIDController orientationController = new PIDController(0.5, 0, 0);

  /**
   * Runs the intake until the sensor is activated
   * @param intake
   */
  public Intake(IntakeSubsystem intake, NoteCamera camera) {
    addRequirements(intake);
    this.intake = intake;
    this.camera = camera;
  }

  public Intake(IntakeSubsystem intake, NoteCamera camera, Rotation2d defaultRotation) {
    this(intake, camera);
    goalOrientation = defaultRotation;
    orientationController.enableContinuousInput(-Math.PI, Math.PI);
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
    return intake.getIntakeSensor() && intake.atTargetAngle();
  }

  @Override
  public Rotation2d getRotationSpeeds() {

    Optional<Double> yawToNote = camera.getYawToNote();
    if (yawToNote.isPresent()) {
      goalOrientation = SwerveDrive.getInstance().getPosition().getRotation().plus(
        new Rotation2d(Math.toRadians(yawToNote.get() / 2))
      );
    }

    return Rotation2d.fromRadians(
      orientationController.calculate(
        SwerveDrive.getInstance().getPosition().getRotation().getRadians(), 
        goalOrientation.getRadians()
      )
    );
  }
}
