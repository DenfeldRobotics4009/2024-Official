// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.ShotProfile;
import frc.robot.auto.util.Field;
import frc.robot.subsystems.AprilTagOdometry;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.SwerveModule;

public class ManualAim extends Command {

  Shooter shooter;
  Controls controls;
  static double angle = 0;

  /** Creates a new Shoot. */
  public ManualAim(
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
    angle = angle+ 10 * controls.steer.getY();
    angle = MathUtil.clamp(angle, Constants.Shooter.aimRangeFrom0, 0);
    //get flywheels are up to speed
    shooter.setPosition(angle);
    shooter.setFlyWheelSpeed(Constants.Shooter.flyWheelSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !shooter.getBarrelSensor();
  }
}