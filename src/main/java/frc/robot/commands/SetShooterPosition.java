// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

public class SetShooterPosition extends Command {

  final IntakeSubsystem intake;
  final Shooter shooter;
  final double shooterPosition;

  /**
   * Sets the intake to the transfer position to allow
   * for shooter movement. The shooter will only be
   * moved when the intake has reached transfer position
   * @param intake
   * @param shooter
   */
  public SetShooterPosition(
    IntakeSubsystem intake, 
    Shooter shooter, 
    double shooterPosition
  ) {
    addRequirements(intake, shooter);

    this.intake = intake;
    this.shooter = shooter;
    this.shooterPosition = shooterPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setPosition(IntakeSubsystem.intakePosition.DEPOSIT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.atTargetAngle()) {
      shooter.setPosition(shooterPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  /**
   * End when the shooter has reached position
   */
  public boolean isFinished() {
    return shooter.atTargetAngle();
  }
}
