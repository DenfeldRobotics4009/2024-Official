// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

public class MoveShooter extends Command {

  final Shooter shooter;
  final double shooterPosition;

  /**
   * Sets the intake to the transfer position to allow
   * for shooter movement. The shooter will only be
   * moved when the intake has reached transfer position
   * @param intake
   * @param shooter
   */
  public MoveShooter(Shooter shooter, double shooterPosition) {
    // addRequirements(intake, shooter);

    this.shooter = shooter;
    this.shooterPosition = shooterPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setPosition(shooterPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
    return shooter.atTargetAngle() && shooter.getTargetAngle() == shooterPosition;
  }
}
