// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Swerve;
import frc.robot.auto.pathing.AutoShuffleboardTab;
import frc.robot.auto.pathing.PathingConstants;
import frc.robot.commands.AmpShoot;
import frc.robot.commands.BurpShoot;
import frc.robot.commands.Climb;
import frc.robot.commands.Drive;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.LowIntake;
import frc.robot.commands.MoveShooter;
import frc.robot.commands.Outtake;
import frc.robot.commands.SetClimberLimits;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootManual;
import frc.robot.commands.TransferIntake;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Climber.climberSide;
import frc.robot.subsystems.Shooter.shooterPosition;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AprilTagOdometry;
import frc.robot.subsystems.Climber;
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
  
  final Shooter shooter = Shooter.getInstance();
  final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  final Climber climber = Climber.getInstance();

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
     * Feeding is triggered by the left trigger
     */
    controls.getOperatePOVTrigger(90).whileTrue(new Shoot(shooter, controls, cam1));

    new Trigger(() -> {return controls.operate.getRightTriggerAxis() >= 0.1;}).whileTrue(
      new FeedShooter(shooter, true)
    );

    /**
     * AMP SHOOT
     * 
     * Moves intake to transfer position to avoid collision,
     * then runs the amp shoot command.
     * 
     * Feeding is triggered by the left trigger
     */
    controls.getOperatePOVTrigger(270).whileTrue(
      new AmpShoot(shooter)
    );

    /**
     * BURP SHOOT
     * 
     * Spins up the flywheels slowly for a short shot.
     * This does not move the intake nor shooter arm
     * 
     * Feeding is triggered by the left trigger
     */
    controls.getOperatePOVTrigger(180).whileTrue(
      new SequentialCommandGroup(
        new BurpShoot(shooter, controls)
      )
    );

    /**
     * INTAKE
     */
    new Trigger(() -> {return controls.operate.getLeftTriggerAxis() >= 0.1;}).whileTrue(
      new TransferIntake(intake, shooter, cam2) // Continue until a piece is picked up
    );


    /**
     * LOW INTAKE
     */
    new Trigger(() -> {
      return controls.operate.getRightStickButton();
    }).whileTrue(
      new LowIntake(intake, cam2)
    );

    /**
     * RESET ARMS
     * 
     * Moves the shooter to 0 position, then moves the intake to the 0 position.
     * This fully tucks all arms into the robot.
     */
    new Trigger(() -> {return controls.operate.getAButton();}).onTrue(
      new MoveShooter(shooter, shooterPosition.GROUND.get())
    );

    /**
     * MANUAL SHOOT
     */
    new Trigger(() -> {return controls.operate.getXButton();}).whileTrue(
      new ShootManual(shooter, -5)
    );

    /**
     * LOW FIELD CROSS SHOT
     */
    new Trigger(() -> {return controls.operate.getLeftStickButton();}).whileTrue(
      new ShootManual(shooter, -145)
    );

    /**
     * TRANSFER
     * 
     * Bypasses the automatic intake process and just runs the transfer.
     */
    // controls.getOperatorButton(6).whileTrue(new Transfer(intake, shooter));

    /**
     * OUTTAKE
     * 
     * Does not move the intake or shooter, only runs the outtake while
     * the button is held.
     * This will outtake the shooter as well, meaning this is not applicable when 2 notes
     * are held in the robot. Instead, the first note should be shot.
     */
    new Trigger(() -> {return controls.operate.getYButton();}).whileTrue(new Outtake(intake, shooter));

    // /**
    //  * FAST INTAKE
    //  * 
    //  * Does not move the intake or shooter, only runs the intake while
    //  * the button is held, or until sensor is tripped.
    //  */
    // new Trigger(() -> {return controls.operate.getBButton();}).whileTrue(new FastIntake(intake));

    /**
     * CLIMBER
     * 
     * Moves the climber arms upward, each arm will automatically stop
     * when its upper bound is reached. The command will not end until
     * the button is released.
     */
    new Trigger(() -> {return controls.operate.getLeftBumper();}).whileTrue(new Climb(climber, Constants.Climber.climberMotorPower));

    /**
     * CLIMBER
     * 
     * Moves the climber arms downward, each arm will automatically stop
     * when its lower bound is reached. The command will not end until
     * the button is released.
     */
    new Trigger(() -> {return controls.operate.getRightBumper();}).whileTrue(new Climb(climber, -Constants.Climber.climberMotorPower));

    /**
     * CLIMBER LIMIT DISABLE
     * 
     * While the B button is held, the climber ignores its limits
     */
    new Trigger(() -> {return controls.operate.getBButton();}).onTrue(new SetClimberLimits(climber, false));
    
    /**
     * CLIMBER LIMIT ENABLE
     * 
     * While the B button is held, the climber ignores its limits
     */
    new Trigger(() -> {return controls.operate.getBButton();}).whileFalse(new SetClimberLimits(climber, true));
    
    /**
     * LEFT CLIMBER UP
     */
    new Trigger(() -> {
      return controls.operate.getBButton() && controls.operate.getLeftY() < -0.1;
    }).whileTrue(new Climb(climber, climberSide.left, Constants.Climber.climberMotorPower));
    
    /**
     * LEFT CLIMBER DOWN
     */
    new Trigger(() -> {
      return controls.operate.getBButton() && controls.operate.getLeftY() > 0.1;
    }).whileTrue(new Climb(climber, climberSide.left, -Constants.Climber.climberMotorPower));
    
    /**
     * RIGHT CLIMBER UP
     */
    new Trigger(() -> {
      return controls.operate.getBButton() && controls.operate.getRightY() < -0.1;
    }).whileTrue(new Climb(climber, climberSide.right, Constants.Climber.climberMotorPower));
    
    /**
     * RIGHT CLIMBER DOWN
     */
    new Trigger(() -> {
      return controls.operate.getBButton() && controls.operate.getRightY() > 0.1;
    }).whileTrue(new Climb(climber, climberSide.right, -Constants.Climber.climberMotorPower));
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
