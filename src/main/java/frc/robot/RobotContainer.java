// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Swerve;
import frc.robot.auto.pathing.AutoShuffleboardTab;
import frc.robot.auto.pathing.PathingConstants;
import frc.robot.commands.Drive;
import frc.robot.odometry.AprilTagOdometry;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  AprilTagOdometry cam2 = new AprilTagOdometry("Microsoft_LifeCam_HD-3000-2", new Transform3d());
  
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
  private void configureBindings() {}

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
