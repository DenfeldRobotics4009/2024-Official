// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteCamera;
import frc.robot.subsystems.Shooter;

public class LowIntake extends Command {
  PIDController aimingPidController = new PIDController(3, 0.1, 0);

  IntakeSubsystem intake;
  NoteCamera camera;

  /**
   * Runs the intake until the sensor is activated
   * @param intake
   */
  public LowIntake(
    IntakeSubsystem intake, 
    NoteCamera camera
  ) {
    addRequirements(intake);
    this.intake = intake;
    this.camera = camera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aimingPidController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    intake.setIntake(Constants.Intake.slowIntakeMotorPower);

    //applyTurnSpeed();
  }

  protected void applyTurnSpeed() {
    double omegaRadPerSecond = 0; // From controller
    Optional<Double> yawToNote = camera.getYawToNote();
    if (yawToNote.isPresent()) {
      omegaRadPerSecond = -aimingPidController.calculate(Math.toRadians(yawToNote.get()));
    }

    Drive.applyTurnSpeed(omegaRadPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    Drive.applyTurnSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getIntakeSensor();
  }
}
