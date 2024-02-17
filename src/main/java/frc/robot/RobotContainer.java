// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Swerve;
import frc.robot.auto.pathing.AutoShuffleboardTab;
import frc.robot.auto.pathing.PathingConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.Intake;
import frc.robot.commands.MoveIntakeFirst;
import frc.robot.commands.MoveShooterFirst;
import frc.robot.commands.Outtake;
import frc.robot.commands.Shoot;
import frc.robot.commands.Transfer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;
import frc.robot.subsystems.Shooter.shooterPosition;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AprilTagOdometry;
import frc.robot.subsystems.NoteCamera;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  final Shooter turret = Shooter.getInstance();
  final IntakeSubsystem intake = IntakeSubsystem.getInstance();

  // Create a new april-tag camera, this is a subsystem.
  public static final AprilTagOdometry cam1 = new AprilTagOdometry(
   new PhotonCamera("Microsoft_LifeCam_HD-3000-1"), Constants.AprilTagOdometry.cameraPose);
  public static final NoteCamera cam2 = new NoteCamera(new PhotonCamera("Microsoft_LifeCam_HD-3000-2"));
  
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
    // TODO FIGURE OUT BUTTON BINDINGS

    /**
     * SHOOT
     * 
     * Moves intake to transfer position to avoid collision,
     * then runs the shoot command to auto aim.
     * 
     * Feeding is triggered by POV UP
     */
    controls.getOperatorButton(1).whileTrue(
      new SequentialCommandGroup(
        new MoveIntakeFirst(intake, turret, intakePosition.DEPOSIT.get(), shooterPosition.GROUND.get()),
        new Shoot(turret, controls, driveTrain, cam1)
      )
    );

    /**
     * INTAKE
     * 
     * Full intake process, canceled when button is released.
     * Intake extends outward -> shooter moves to transfer -> intake motors run
     * -> move shooter to transfer (will end immediately) -> move intake to transfer
     * -> run transfer process.
     */
    controls.getOperatorButton(2).whileTrue(
      new SequentialCommandGroup(
        new MoveIntakeFirst(intake, turret, intakePosition.GROUND.get(), shooterPosition.DEPOSIT.get()),
        new Intake(intake), // Continue until a piece is picked up
        // Initiate transfer positions
        new MoveShooterFirst(intake, turret, intakePosition.DEPOSIT.get(), shooterPosition.DEPOSIT.get()),
        new Transfer(turret, intake),
        new MoveShooterFirst(intake, turret, intakePosition.STARTING.get(), shooterPosition.GROUND.get())
      )
    );

    /**
     * RESET ARMS
     * 
     * Moves the shooter to 0 position, then moves the intake to the 0 position.
     * This fully tucks all arms into the robot.
     */
    controls.getOperatorButton(4).onTrue(
      new MoveShooterFirst(intake, turret, intakePosition.STARTING.get(), shooterPosition.GROUND.get())
    );

    /**
     * TRANSFER
     * 
     * Bypasses the automatic intake process and just runs the transfer.
     */
    controls.getOperatorButton(6).whileTrue(new Transfer(turret, intake));

    /**
     * OUTTAKE
     * 
     * Does not move the intake or shooter, only runs the outtake while
     * the button is held.
     */
    controls.getOperatorButton(5).whileTrue(new Outtake(intake));

    /**
     * INTAKE
     * 
     * Does not move the intake or shooter, only runs the intake while
     * the button is held, or until sensor is tripped.
     */
    controls.getOperatorButton(3).whileTrue(new Intake(intake));
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
