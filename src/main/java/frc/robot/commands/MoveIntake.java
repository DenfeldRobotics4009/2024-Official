// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

public class MoveIntake extends Command {

  final IntakeSubsystem intake;
  final double intakePosition;

  /**
   * Sets the intake to the transfer position to allow
   * for shooter movement.
   * @param intake
   * @param shooter
   */
  public MoveIntake(
    IntakeSubsystem intake, 
    double intakePosition
  ) {
    // addRequirements(intake, shooter);

    this.intake = intake;
    this.intakePosition = intakePosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setPosition(intakePosition);
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
    return intake.atTargetAngle() && intake.getTargetAngle() == intakePosition;
  }
}
