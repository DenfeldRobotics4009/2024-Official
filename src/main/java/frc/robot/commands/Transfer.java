// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.IntakeArm.positionOptions;

public class Transfer extends Command {
  final Turret turret;
  final IntakeArm intake;

  /** Creates a new Transfer. */
  public Transfer(Turret turret, IntakeArm intake) {
    addRequirements(turret, intake);

    this.turret = turret;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setPosition(positionOptions.DEPOSIT);
    turret.setAngle(Constants.Turret.transferAngle);
    if (turret.atTargetAngle()) {
      intake.setIntake(-0.8);
      turret.feed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    turret.stopFeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turret.getBarrelSensor();
  }
}
