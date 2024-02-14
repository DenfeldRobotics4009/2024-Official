// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

public class SetArmPositions extends Command {

  final IntakeSubsystem intake;
  final Shooter shooter;
  final double intakePosition;
  final double shooterPosition;

  /**
   * Sets the intake to the transfer position to allow
   * for shooter movement. The shooter will only be
   * moved when the intake has reached transfer position
   * @param intake
   * @param shooter
   */
  public SetArmPositions(
    IntakeSubsystem intake, 
    Shooter shooter, 
    double intakePosition,
    double shooterPosition
  ) {
    addRequirements(intake, shooter);

    this.intake = intake;
    this.shooter = shooter;
    this.intakePosition = intakePosition;
    this.shooterPosition = shooterPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setPosition(intakePosition);
    shooter.setPosition(shooterPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  /**
   * End when the intake has reached position
   */
  public boolean isFinished() {
    return (
      intake.atTargetAngle() && intake.getTargetAngle() == intakePosition &&
      shooter.atTargetAngle() && shooter.getTargetAngle() == shooterPosition
    );
  }
}
