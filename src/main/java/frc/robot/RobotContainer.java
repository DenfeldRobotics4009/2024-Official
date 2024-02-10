// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Swerve;
import frc.robot.auto.pathing.AutoShuffleboardTab;
import frc.robot.auto.pathing.PathingConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Intake;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AprilTagOdometry;
import frc.robot.subsystems.NoteCamera;
import frc.robot.subsystems.SwerveDrive;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  final Turret turret = Turret.getInstance();
  final IntakeArm intake = IntakeArm.getInstance();

  // Create a new april-tag camera, this is a subsystem.
  final AprilTagOdometry cam1 = new AprilTagOdometry(
    new PhotonCamera("Microsoft_LifeCam_HD-3000-1"), Constants.AprilTagOdometry.cameraPose);
  final NoteCamera cam2 = new NoteCamera(new PhotonCamera("Microsoft_LifeCam_HD-3000-2"));
  
  final Controls controls = Controls.getInstance();
  final SwerveDrive driveTrain = SwerveDrive.getInstance();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    driveTrain.setDefaultCommand(
      new Drive(driveTrain, controls)
    );

    // Pass drivetrain into pathing algorithm
    PathingConstants.setForwardAngle(Swerve.forwardAngle);
    PathingConstants.setDriveSubsystem(driveTrain);
    // Initialize auto tab
    AutoShuffleboardTab.getInstance();

    // Configure the button bindings

    driveTrain.setDefaultCommand(
      new Drive(driveTrain, controls)
    );

    // Pass drivetrain into pathing algorithm
    PathingConstants.setForwardAngle(Swerve.forwardAngle);
    PathingConstants.setDriveSubsystem(driveTrain);
    // Initialize auto tab
    AutoShuffleboardTab.getInstance();

    // Configure the button bindings
    configureBindings();
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //controls.getOperatorButton(1).whileTrue(new Shoot(turret, controls, driveTrain));

    controls.getOperatorButton(2).whileTrue(new Intake(intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return AutoShuffleboardTab.getInstance().getSelectedAuto();
  }
}
