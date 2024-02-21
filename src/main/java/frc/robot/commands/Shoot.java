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

public class Shoot extends Command {

  Shooter shooter;
  Controls controls;
  SwerveDrive swerveDrive;
  AprilTagOdometry camera;

  double angle = 0;

  PIDController aimingPidController = new PIDController(6, 0.1, 0);

  /** Creates a new Shoot. */
  public Shoot(
    Shooter shooter, 
    Controls controls, 
    SwerveDrive swerveDrive,
    AprilTagOdometry camera
  ) {
    this.shooter = shooter;
    this.controls = controls;
    this.swerveDrive = swerveDrive;
    this.camera = camera;
    
    addRequirements(shooter, swerveDrive);

    aimingPidController.setSetpoint(0);
    aimingPidController.enableContinuousInput(-Math.PI, Math.PI);
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
    System.out.println("Distance: "+distance);
    // Convert joystick value into a shooter angle
    double angle = shooter.getTargetAngle();
    if (ShotProfile.getHeightFromDistance(distance).isPresent()) {
      angle = ShotProfile.getHeightFromDistance(distance).get();
    }

    angle += 3* controls.operate.getLeftY();

    System.out.println("Barrel Sensor " + shooter.getBarrelSensor());
    // System.out.println("Distance " + distance);
    // System.out.println("Shot angle " + angle);
    // System.out.println("Yaw " + camera.getYawToSpeaker());

    //get flywheels are up to speed
    shooter.setPosition(angle);
    boolean atShooterSpeed = shooter.setFlyWheelSpeed(Constants.Shooter.flyWheelSpeed);

    //aim drive train
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      new ChassisSpeeds(
        controls.getForward() * SwerveModule.maxMetersPerSecond,
        controls.getLateral() * SwerveModule.maxMetersPerSecond,
        aimingPidController.calculate(Math.toRadians(camera.getYawToSpeaker()-5))
      ), 
      SwerveDrive.navxGyro.getRotation2d()
    );
    System.out.println(speeds.omegaRadiansPerSecond);
    swerveDrive.drive(speeds);
    //if flywheels up to speed, shooter aimed, drive train aimed, then feed in
    // if (controls.operate.getRightTriggerAxis() >= 0.1 && atShooterSpeed) {
    //   shooter.feed();
    // }
    // else {
    //   shooter.stopFeed();
    // }
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
