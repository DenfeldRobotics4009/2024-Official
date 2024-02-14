// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

public class SetIntakePosition extends Command {

  final IntakeSubsystem intake;
  final Shooter shooter;
  final double intakePosition;

  /**
   * Sets the intake to the transfer position to allow
   * for shooter movement. The shooter will only be
   * moved when the intake has reached transfer position
   * @param intake
   * @param shooter
   */
  public SetIntakePosition(
    IntakeSubsystem intake, 
    Shooter shooter, 
    double intakePosition
  ) {
    addRequirements(intake, shooter);

    this.intake = intake;
    this.shooter = shooter;
    this.intakePosition = intakePosition;

    System.out.println("Position: " + intakePosition);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.atTargetAngle()) {
      intake.setPosition(intakePosition);
    }
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
