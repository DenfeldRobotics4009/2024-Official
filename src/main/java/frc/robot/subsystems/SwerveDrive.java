// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import frc.robot.auto.pathing.DriveSubsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveMotors;

public class SwerveDrive extends SubsystemBase implements DriveSubsystem {

  public final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
  final Field2d fieldWidget = new Field2d();

  // Construct swerve modules
    // Pass swerve tab into modules to allow each of them to display relevant 
    // data, whatever that may be
  final SwerveModule FrontLeftModule = new SwerveModule(
    new SwerveMotors(
      Swerve.FrontLeft.driveID, 
      Swerve.FrontLeft.steerID, 
      Swerve.FrontLeft.CANCoderID
    ), 
    Swerve.FrontLeft.trackPosition, 
    Swerve.FrontLeft.class.getSimpleName()
  );
  final SwerveModule FrontRightModule = new SwerveModule(
    new SwerveMotors(
      Swerve.FrontRight.driveID, 
      Swerve.FrontRight.steerID, 
      Swerve.FrontRight.CANCoderID
    ), 
    Swerve.FrontRight.trackPosition, 
    Swerve.FrontRight.class.getSimpleName()
  );
  final SwerveModule BackLeftModule = new SwerveModule(
    new SwerveMotors(
      Swerve.BackLeft.driveID, 
      Swerve.BackLeft.steerID, 
      Swerve.BackLeft.CANCoderID
    ), 
    Swerve.BackLeft.trackPosition, 
    Swerve.BackLeft.class.getSimpleName()
  );
  final SwerveModule BackRightModule = new SwerveModule(
    new SwerveMotors(
      Swerve.BackRight.driveID, 
      Swerve.BackRight.steerID, 
      Swerve.BackRight.CANCoderID
    ), 
    Swerve.BackRight.trackPosition, 
    Swerve.BackRight.class.getSimpleName()
  );

  SwerveDriveKinematics kinematics;

  public static AHRS navxGyro = new AHRS();

  // Entries for motion graphing
  GenericEntry xPositionEntry = swerveTab.add("xPosition", 0
    ).withPosition(3, 4).withSize(2, 1).getEntry();
  GenericEntry  yPositionEntry = swerveTab.add("yPosition", 0
    ).withPosition(5, 4).withSize(2, 1).getEntry();
  GenericEntry  rotationEntry = swerveTab.add("Rotation", 0
    ).withPosition(3, 0).withSize(4, 4).withWidget("Gyro").getEntry();

  GenericEntry  xVelocityEntry = swerveTab.add("xVelocity", 0
    ).withPosition(0, 4).withSize(3, 1).getEntry();
  GenericEntry  yVelocityEntry = swerveTab.add("yVelocity", 0
    ).withPosition(0, 5).withSize(3, 1).getEntry();
  GenericEntry  rotationVelocityEntry = swerveTab.add("rotationVelocity", 0
    ).withPosition(0, 0).withSize(3, 4).withWidget("Gyro").getEntry();

  /**
   * Object to track the robots position via inverse kinematics
   */
  private SwerveDrivePoseEstimator robotPoseEstimator;

  static SwerveDrive instance;

  public static SwerveDrive getInstance() {
    if (instance == null) {
      instance = new SwerveDrive();
    }
    return instance;
  }

  /** Creates a new SwerveDrive. */
  private SwerveDrive() {

    // Initialize during constructor to avoid building kinematics
    // object with uninitialized swerve modules.
    kinematics = new SwerveDriveKinematics(
      // Same order as initialization
      SwerveModule.getTrackPositions()
    );

    navxGyro.setAngleAdjustment(-Swerve.forwardAngle.getDegrees());

    /**
     * Define with default values, this may be rebuilt when the auto
     * runs to account for choosing an auto position.
     */
    setPosition(new Pose2d());

    robotPoseEstimator.resetPosition(navxGyro.getRotation2d(), getModulePositions(), new Pose2d());

    // Construct field widget
    swerveTab.add("Robot Position", fieldWidget
      ).withPosition(7, 0).withSize(18, 10);
  }

  /**
   * @return distance and angle each swerve module has driven
   */
  SwerveModulePosition[] getModulePositions() {
    ArrayList<SwerveModulePosition> positions = new ArrayList<SwerveModulePosition>();
    for (SwerveModule swerveModule : SwerveModule.instances) {
      positions.add(
        new SwerveModulePosition(
          // grab current drive distance and direction
          swerveModule.swerveMotors.getDriveDistanceMeters(), swerveModule.swerveMotors.getRotation2d()
        )
      );
    }
    // Pass out as array
    return positions.toArray(new SwerveModulePosition[0]);
  }

  @Override
  public void periodic() {
    // Update pose estimation
    robotPoseEstimator.updateWithTime(Timer.getFPGATimestamp(), navxGyro.getRotation2d(), getModulePositions());

    // Displaying position values
    xPositionEntry.setDouble(robotPoseEstimator.getEstimatedPosition().getX());
    yPositionEntry.setDouble(robotPoseEstimator.getEstimatedPosition().getY());
    rotationEntry.setDouble(robotPoseEstimator.getEstimatedPosition().getRotation().getDegrees());

    fieldWidget.setRobotPose(
      new Pose2d(
        getPosition().getTranslation().plus(new Translation2d(1.8, 0.4)), new Rotation2d()
      )
    );
  }

  /**
   * Drives the robot in a robot oriented manner, if field oriented is
   * desired, inputs must be rotated by calling function accordingly.
   * @param Speeds Robot relative chassis speeds on a scale from 0 to 1.
   */
  public void drive(ChassisSpeeds Speeds) {

    xVelocityEntry.setDouble(Speeds.vxMetersPerSecond);
    yVelocityEntry.setDouble(Speeds.vyMetersPerSecond);
    rotationVelocityEntry.setDouble(Math.toDegrees(Speeds.omegaRadiansPerSecond));

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(Speeds);
    for (int i = 0; i < 4; i++) {
      SwerveModule.instances.get(i).drive(states[i]);
    }
  }

  public Pose2d getPosition() {
    return robotPoseEstimator.getEstimatedPosition();
  }

  // /**
  //  * Grabs velocity from generic entry table,
  //  * not from sensor collections
  //  * @return A Pose2d with translation values bounded from -1 to 1
  //  */
  // public Pose2d getVelocity() {
  //   return velocity;
  // }

  public void setPosition(Pose2d position) {
    // Rebuild pose estimator with more relevant values
    robotPoseEstimator = new SwerveDrivePoseEstimator(
      kinematics, 
      navxGyro.getRotation2d(), 
      getModulePositions(),
      position
    );
  }

  public void addVisionMeasurement(Pose2d visionPosition, double timestampSeconds) {
    robotPoseEstimator.addVisionMeasurement(visionPosition, timestampSeconds);
  }
}
