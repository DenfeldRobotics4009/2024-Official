// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.positionOptions;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command {
Controls controls;
Intake intake;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCommand(Controls controls, Intake intake) {
    this.controls = controls;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(controls.drive.getRawButton(7)){
      intake.setPosition(positionOptions.SOURCE);
    }
    if(controls.drive.getRawButton(9)){
      intake.setPosition(positionOptions.DEPOSIT);
      }
    if(controls.drive.getRawButton(11)){
      intake.setPosition(positionOptions.GROUND);
    }
    if(controls.drive.getRawButton(2)){
      intake.setIntake();
    }
    if(controls.drive.getRawButton(1)){
      intake.setOutake();
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
