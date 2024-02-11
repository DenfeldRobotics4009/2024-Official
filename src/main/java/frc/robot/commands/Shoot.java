// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Optional;

import org.opencv.photo.Photo;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.auto.util.Field;
import frc.robot.subsystems.AprilTagOdometry;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.IntakeArm.positionOptions;
import frc.robot.subsystems.swerve.SwerveModule;

public class Shoot extends Command {

  Turret turret;
  Controls controls;
  SwerveDrive swerveDrive;
  AprilTagOdometry camera;
  IntakeArm intake;

  PIDController aimingPidController = new PIDController(0.1, 0, 0);

  /** Creates a new Shoot. */
  public Shoot(
    Turret turret, 
    Controls controls, 
    SwerveDrive swerveDrive,
    AprilTagOdometry camera,
    IntakeArm intake
  ) {
    this.turret = turret;
    this.controls = controls;
    this.swerveDrive = swerveDrive;
    this.camera = camera;
    this.intake = intake;
    addRequirements(turret, swerveDrive, intake);

    aimingPidController.setSetpoint(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setPosition(positionOptions.DEPOSIT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Grab target position from april tag
    Translation2d targetPose = new Translation2d();
    int speakerID;
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      speakerID = 4;
    } else {
      speakerID = 7;
    }
    targetPose = AprilTagOdometry.aprilTagFieldLayout.getTagPose(speakerID).get().getTranslation().toTranslation2d();
    targetPose = Field.translateRobotPoseToRed(targetPose);
    // Calculate distance
    double distance = targetPose.getDistance(swerveDrive.getPosition().getTranslation());
    
    //get flywheels are up to speed
    boolean atShooterSpeed = turret.setFlyWheelSpeed(Constants.Turret.flyWheelSpeed);
    //aim shooter
    double angle = -controls.operate.getThrottle() * Constants.Turret.aimRangeFrom0; // todo: implement april tags
    turret.setAngle(angle);

    SmartDashboard.putNumber("Distance to target", distance);
    SmartDashboard.putNumber("Shooter Angle", angle);

    double turn = controls.getTurn() * SwerveModule.maxRadPerSecond;

    Optional<PhotonTrackedTarget> target = Optional.empty();
    List<PhotonTrackedTarget> targets = camera.getTargets();
    for (PhotonTrackedTarget photonTrackedTarget : targets) {
      if (photonTrackedTarget.getFiducialId() == speakerID) {
        target = Optional.of(photonTrackedTarget);
      }
    }
    if (target.isPresent()) {
      turn = -aimingPidController.calculate(target.get().getYaw() + -5);
    }

    //aim drive train
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      new ChassisSpeeds(
        controls.getForward() * SwerveModule.maxMetersPerSecond,
        controls.getLateral() * SwerveModule.maxMetersPerSecond,
        turn
      ), 
      SwerveDrive.navxGyro.getRotation2d()
    );
    swerveDrive.drive(speeds);
    //if flywheels up to speed, shooter aimed, drive train aimed, then feed in
    if (atShooterSpeed && turret.atTargetAngle() && controls.getOperatorButton(4).getAsBoolean()) {
      turret.feed();
    }
    else {
      turret.stopFeed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
