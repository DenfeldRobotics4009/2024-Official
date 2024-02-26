// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;

public class Transfer extends Command {
  final Shooter shooter;
  final IntakeSubsystem intake;

  /**
   * Runs the transfer from intake to shooter, this
   * command should run after the positions of the
   * shooter and intake are set.
   * @param shooter
   * @param intake
   */
  public Transfer(IntakeSubsystem intake, Shooter shooter) {
    //addRequirements(shooter, intake);

    this.shooter = shooter;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntake(-1);
    shooter.feed();
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    shooter.stopFeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.getBarrelSensor();
  }
}
