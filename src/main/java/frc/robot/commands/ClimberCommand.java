// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.Climber.positionOptions;

/** An example command that uses an example subsystem. */
public class ClimberCommand extends Command {
Controls controls;
Climber climber;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberCommand(Controls controls, Climber climber) {
    this.controls = controls;
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(controls.drive.getRawButton(6)){
      climber.setPosition(positionOptions.DOWN);
    }
    if(controls.drive.getRawButton(5)){
      climber.setPosition(positionOptions.UP);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
