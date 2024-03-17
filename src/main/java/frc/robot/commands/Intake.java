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
import frc.robot.auto.util.Field;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteCamera;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

public class Intake extends TransferIntake implements RotationController {

  Rotation2d goalOrientation;
  PIDController orientationController = new PIDController(4, 0, 0);

  /**
   * Runs the intake until the sensor is activated
   * @param intake
   */
  public Intake(IntakeSubsystem intake, NoteCamera camera) {
    super(intake, Shooter.getInstance(), camera);
  }

  public Intake(IntakeSubsystem intake, NoteCamera camera, Rotation2d defaultRotation) {
    super(intake, Shooter.getInstance(), camera);
    if (Field.isRedAlliance()) defaultRotation = new Rotation2d().minus(defaultRotation);
    goalOrientation = defaultRotation;
    orientationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntake();
    shooter.feed();
  }
  
  @Override
  public Rotation2d getRotationSpeeds() {

    //Optional<Double> yawToNote = camera.getYawToNote();
    // if (yawToNote.isPresent()) {
    //   goalOrientation = SwerveDrive.getInstance().getPosition().getRotation().plus(
    //     new Rotation2d(Math.toRadians(yawToNote.get()))
    //   );
    // }

    return Rotation2d.fromRadians(
      -orientationController.calculate(
        SwerveDrive.getInstance().getPosition().getRotation().getRadians(), 
        goalOrientation.getRadians()
      )
    );
  }
}
