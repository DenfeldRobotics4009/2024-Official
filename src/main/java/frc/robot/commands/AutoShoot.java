// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.ShotProfile;
import frc.robot.auto.pathing.AutoRotationSource;
import frc.robot.subsystems.AprilTagOdometry;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;
import frc.robot.subsystems.swerve.SwerveModule;

public class AutoShoot extends Command implements AutoRotationSource {

  Shooter shooter;
  AprilTagOdometry camera;
  Rotation2d shootAngle;

  PIDController aimingPidController = new PIDController(0.1, 0, 0);

  /** Creates a new Shoot. */
  public AutoShoot(
    Shooter shooter, 
    AprilTagOdometry camera
  ) {
    this.shooter = shooter;
    this.camera = camera;
    
    addRequirements(shooter);

    aimingPidController.setSetpoint(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate distance
    double distance = camera.getDistanceToSpeaker();
    // Convert joystick value into a shooter angle
    double angle = 0; //TODO: set default
    if (ShotProfile.getHeightFromDistance(distance).isPresent()) {
      angle = ShotProfile.getHeightFromDistance(distance).get();
    }
    SmartDashboard.putNumber("Distance", distance);
    SmartDashboard.putNumber("Angle Shot", angle);

    //get flywheels are up to speed
    shooter.setPosition(angle);
    boolean atShooterSpeed = shooter.setFlyWheelSpeed(Constants.Shooter.flyWheelSpeed);

    //aim drive train

    
    //if flywheels up to speed, shooter aimed, drive train aimed, then feed in
    if (atShooterSpeed && shooter.atTargetAngle()) { //TODO: How to check if chassis is facing correct angle?
      shooter.feed();
    }
    else {
      shooter.stopFeed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter(); //TODO: Should we stop Shooter here? piece might not be fully out of barrel.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !shooter.getBarrelSensor();
  }

  @Override
  public Optional<Rotation2d> getGoalRotation() {
    return Optional.of(shootAngle);
  }
}
