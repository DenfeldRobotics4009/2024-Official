// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import frc.robot.commands.MoveShooterFirst;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;
import frc.robot.subsystems.Shooter.shooterPosition;

public class CloseArms extends MoveShooterFirst {
  /**
   * Lowers the intake to the ground, and sets the shooter
   * to the deposit position.
   */
  public CloseArms() {
    super(
      IntakeSubsystem.getInstance(), 
      Shooter.getInstance(), 
      intakePosition.GROUND.get(), 
      shooterPosition.DEPOSIT.get()
    );
  }
}
