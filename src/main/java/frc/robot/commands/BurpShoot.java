// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.ShotProfile;
import frc.robot.subsystems.AprilTagOdometry;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;
import frc.robot.subsystems.swerve.SwerveModule;

public class BurpShoot extends Command {

  Shooter shooter;
  Controls controls;

  /** Creates a new Shoot. */
  public BurpShoot(
    Shooter shooter, 
    Controls controls
  ) {
    this.shooter = shooter;
    this.controls = controls;
    
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setFlyWheelSpeed(
      Constants.Shooter.topAmpFlyWheelSpeed, 
      Constants.Shooter.bottomAmpFlyWheelSpeed
    );

    //if flywheels up to speed, shooter aimed, drive train aimed, then feed in
    if (controls.operate.getRightTriggerAxis() >= 0.1) {
      shooter.feed();
    }
    else {
      shooter.stopFeed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
