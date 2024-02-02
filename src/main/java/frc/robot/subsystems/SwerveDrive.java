// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import frc.robot.auto.pathing.DriveSubsystem;
import frc.robot.odometry.AprilTagOdometry;
import frc.robot.odometry.OdometryHandler;
import frc.robot.odometry.OdometrySource;
import frc.robot.odometry.SwerveDriveInverseKinematics;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveMotors;

public class SwerveDrive extends SubsystemBase implements DriveSubsystem {

  final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
  final Field2d fieldWidget = new Field2d();

  AprilTagOdometry cam1 = new AprilTagOdometry("Microsoft_LifeCam_HD-3000-1", new Transform3d());

  Pose2d velocity = new Pose2d();

  // Construct swerve modules
  final SwerveModule
    // Pass swerve tab into modules to allow each of them to display relevant 
    // data, whatever that may be
    FrontLeftModule = new SwerveModule(
      new SwerveMotors(
        Swerve.FrontLeft.driveID, 
        Swerve.FrontLeft.steerID, 
        Swerve.FrontLeft.CANCoderID, 
        Swerve.FrontLeft.defaultCalibration
      ), 
      Swerve.FrontLeft.trackPosition, 
      Swerve.FrontLeft.class.getSimpleName()
    ),
    FrontRightModule = new SwerveModule(
      new SwerveMotors(
        Swerve.FrontRight.driveID, 
        Swerve.FrontRight.steerID, 
        Swerve.FrontRight.CANCoderID, 
        Swerve.FrontRight.defaultCalibration
      ), 
      Swerve.FrontRight.trackPosition, 
      Swerve.FrontRight.class.getSimpleName()
    ),
    BackLeftModule = new SwerveModule(
      new SwerveMotors(
        Swerve.BackLeft.driveID, 
        Swerve.BackLeft.steerID, 
        Swerve.BackLeft.CANCoderID, 
        Swerve.BackLeft.defaultCalibration
      ), 
      Swerve.BackLeft.trackPosition, 
      Swerve.BackLeft.class.getSimpleName()
    ),
    BackRightModule = new SwerveModule(
      new SwerveMotors(
        Swerve.BackRight.driveID, 
        Swerve.BackRight.steerID, 
        Swerve.BackRight.CANCoderID, 
        Swerve.BackRight.defaultCalibration
      ), 
      Swerve.BackRight.trackPosition, 
      Swerve.BackRight.class.getSimpleName()
    );

  SwerveDriveKinematics kinematics;

  public static AHRS navxGyro = new AHRS();

  // Entries for motion graphing
  GenericEntry 
    xPositionEntry = swerveTab.add("xPosition", 0
      ).withPosition(3, 4).withSize(2, 1).getEntry(), 
    yPositionEntry = swerveTab.add("yPosition", 0
      ).withPosition(5, 4).withSize(2, 1).getEntry(), 
    rotationEntry = swerveTab.add("Rotation", 0
      ).withPosition(3, 0).withSize(4, 4).withWidget("Gyro").getEntry(),

    xVelocityEntry = swerveTab.add("xVelocity", 0
      ).withPosition(0, 4).withSize(3, 1).getEntry(), 
    yVelocityEntry = swerveTab.add("yVelocity", 0
      ).withPosition(0, 5).withSize(3, 1).getEntry(), 
    rotationVelocityEntry = swerveTab.add("rotationVelocity", 0
      ).withPosition(0, 0).withSize(3, 4).withWidget("Gyro").getEntry();

  /**
   * Object to track the robots position via inverse kinematics
   */
  public static SwerveDriveInverseKinematics inverseKinematics;

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

    inverseKinematics = SwerveDriveInverseKinematics.getInstance(navxGyro);

    navxGyro.setAngleAdjustment(-Swerve.forwardAngle.getDegrees());

    // Construct field widget
    swerveTab.add("Robot Position", fieldWidget
      ).withPosition(7, 0).withSize(18, 10);
  }

  @Override
  public void periodic() {

    inverseKinematics.periodic();

    // Displaying position values
    xPositionEntry.setDouble(OdometryHandler.getBestPosition().getX());
    yPositionEntry.setDouble(OdometryHandler.getBestPosition().getY());
    rotationEntry.setDouble(navxGyro.getRotation2d().getDegrees());

    fieldWidget.setRobotPose(
      new Pose2d(
        getPosition().getTranslation().plus(new Translation2d(1.8, 0.4)), new Rotation2d()
      )
    );
  }

  /**
   * Drives the robot in a robot oriented manner, if field oriented is
   * desired, inputs must be rotated by calling function accordingly.
   * @param speeds Robot relative chassis speeds on a scale from 0 to 1.
   */
  public void drive(ChassisSpeeds speeds) {

    xVelocityEntry.setDouble(speeds.vxMetersPerSecond);
    yVelocityEntry.setDouble(speeds.vyMetersPerSecond);
    rotationVelocityEntry.setDouble(Math.toDegrees(speeds.omegaRadiansPerSecond));

    velocity = new Pose2d(
      speeds.vxMetersPerSecond, 
      speeds.vyMetersPerSecond, 
      new Rotation2d(speeds.omegaRadiansPerSecond)
    );

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    for (int i = 0; i < 4; i++) {
      SwerveModule.instances.get(i).drive(states[i]);
    }
  }

  public Pose2d getPosition() {
    return OdometryHandler.getBestPosition();
  }

  /**
   * Grabs velocity from generic entry table,
   * not from sensor collections
   * @return A Pose2d with translation values bounded from -1 to 1
   */
  public Pose2d getVelocity() {
    return velocity;
  }

  public void setPosition(Pose2d position) {
    inverseKinematics.setPosition(position);
  }
}
