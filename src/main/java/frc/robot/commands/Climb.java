// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.climberSide;

public class Climb extends Command {
  final Climber climber;
  climberSide side;
  double speed;

  /**
   * Moves the climbers up
   */
  public Climb(Climber climber, double speed) {
    addRequirements(climber);

    this.climber = climber;
    side = climberSide.both;
    this.speed = speed;
  }

  /**
   * Moves the climbers up
   */
  public Climb(Climber climber, climberSide side,  double speed) {
    addRequirements(climber);

    this.climber = climber;
    this.side = side;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (side == climberSide.both || side == climberSide.left) {
      climber.moveLeftClimber(speed);
    }

    if (side == climberSide.both || side == climberSide.right) {
      climber.moveRightClimber(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
