// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class FeedShooter extends Command {
  Shooter shooter;
  boolean waitForShooter;

  public FeedShooter(Shooter shooter, boolean waitForShooter) {
    this.shooter = shooter;
    this.waitForShooter = waitForShooter;
  }

  public FeedShooter(Shooter shooter) {
    this.shooter = shooter;
    this.waitForShooter = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (waitForShooter) {
      if (shooter.atFlyWheelSpeed()) shooter.feed();
    } else {
      shooter.feed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopFeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
