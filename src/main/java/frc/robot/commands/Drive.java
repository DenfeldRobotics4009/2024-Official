// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModule;

public class Drive extends Command {
  SwerveDrive drivetrain;
  Controls controls;

  // Tuner values are in degrees, and are converted after calculation
  PIDController directionTuner = new PIDController(4, 0, 0);

  /** Creates a new Drive. */
  public Drive(SwerveDrive Drivetrain, Controls Controls) {
    addRequirements(Drivetrain);

    drivetrain = Drivetrain;
    controls = Controls;

    directionTuner.enableContinuousInput(0, 360);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double radPSec;

    double precisionFactor = 0.75;
    if (controls.getPrecisionMode()) {precisionFactor = 0.35;}

    // If the hat is held in a direction
    if (controls.getPOV() != -1) {
      // Units are in degrees for POV hat, thus use degrees from navx
      radPSec = MathUtil.clamp(
          Math.toRadians( directionTuner.calculate(
            SwerveDrive.navxGyro.getRotation2d().times(-1).getDegrees(), 
            controls.getPOV()
          )),
          -SwerveModule.maxRadPerSecond, SwerveModule.maxRadPerSecond
        );
    } else {
      radPSec = controls.getTurn() * SwerveModule.maxRadPerSecond;
    }

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      new ChassisSpeeds(
        controls.getForward() * SwerveModule.maxMetersPerSecond * precisionFactor,
        controls.getLateral() * SwerveModule.maxMetersPerSecond * precisionFactor,
        radPSec * precisionFactor
      ), 
      SwerveDrive.navxGyro.getRotation2d()
    );
    drivetrain.drive(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
