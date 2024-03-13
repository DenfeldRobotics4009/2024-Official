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

public class Shoot extends Command {

  Shooter shooter;
  Controls controls;
  AprilTagOdometry camera;

  double offset = 0;

  PIDController aimingPidController = new PIDController(6, 0.1, 0);

  /** Creates a new Shoot. */
  public Shoot(
    Shooter shooter, 
    Controls controls, 
    AprilTagOdometry camera
  ) {
    this.shooter = shooter;
    this.controls = controls;
    this.camera = camera;
    
    addRequirements(shooter);

    aimingPidController.setSetpoint(0);
    aimingPidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    offset = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Calculate distance
    double distance = camera.getDistanceToSpeaker();
    SmartDashboard.putNumber("Distance: ", distance);
    // Convert joystick value into a shooter angle
    double angle = 0;
    if (ShotProfile.getHeightFromDistance(distance).isPresent()) {
      angle = ShotProfile.getHeightFromDistance(distance).get();
    }
    angle +=  offset += Controls.modifyAxis(3* controls.operate.getLeftY(), 0.6);

    // System.out.println("Distance " + distance);
    // System.out.println("Shot angle " + angle);
    SmartDashboard.putNumber("Yaw ", camera.getYawToSpeaker());

    //get flywheels are up to speed
    shooter.setPosition(angle);
    boolean atShooterSpeed = shooter.setFlyWheelSpeed(Constants.Shooter.flyWheelSpeed);

    SmartDashboard.putNumber("Angle ", angle);
    SmartDashboard.putNumber("Offset ", offset);
    SmartDashboard.putNumber("Offset + Angle", angle + offset);

    // Contact the driving command and apply an external turning speed towards the target
    Drive.applyTurnSpeed(aimingPidController.calculate(Math.toRadians(camera.getYawToSpeaker())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    Drive.applyTurnSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
