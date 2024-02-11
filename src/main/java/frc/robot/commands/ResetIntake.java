// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.IntakeArm.positionOptions;

public class ResetIntake extends Command {
  IntakeArm intake;
  Turret turret;
  /** Creates a new ResetIntake. */
  public ResetIntake(IntakeArm intake, Turret turret) {
    addRequirements(intake, turret);
    this.intake = intake;
    this.turret = turret;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setAngle(0);
    if (turret.atTargetAngle()) {
      intake.setPosition(positionOptions.STARTING);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}