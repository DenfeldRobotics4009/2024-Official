// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteCamera;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shooterPosition;

public class TransferIntake extends LowIntake {

  Shooter shooter;

  /**
   * Runs the intake until the sensor is activated
   * @param intake
   */
  public TransferIntake(
    IntakeSubsystem intake, 
    Shooter shooter,
    NoteCamera camera
  ) {

    super(intake, camera);
    addRequirements(shooter);
    this.intake = intake;
    this.camera = camera;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setPosition(shooterPosition.DEPOSIT);
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntake();
    shooter.feed();
    applyTurnSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopFeed();
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.getBarrelSensor();
  }
}
