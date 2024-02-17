// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetFlywheelSpeed extends Command {
  Shooter shooter;
  double topFlyWheelRPM;
  double bottomFlyWheelRPM;

  public SetFlywheelSpeed(Shooter shooter, double flyWheelRPM) {
    this(shooter, flyWheelRPM, flyWheelRPM);
  }

  public SetFlywheelSpeed(Shooter shooter, double topFlyWheelRPM, double bottomFlyWheelRPM) {
    addRequirements(shooter);
    this.shooter = shooter;
    this.topFlyWheelRPM = topFlyWheelRPM;
    this.bottomFlyWheelRPM = bottomFlyWheelRPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setFlyWheelSpeed(topFlyWheelRPM, bottomFlyWheelRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
