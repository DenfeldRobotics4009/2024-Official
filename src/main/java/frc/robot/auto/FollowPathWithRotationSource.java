// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.Optional;

import javax.swing.text.html.Option;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.pathing.AutoRotationSource;
import frc.robot.auto.pathing.AutoShuffleboardTab;
import frc.robot.auto.pathing.FollowPath;
import frc.robot.auto.pathing.PathingConstants;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathState;

public class FollowPathWithRotationSource extends FollowPath {
  final Command command;

  PIDController rotationController = new PIDController(PathingConstants.turningProportion, 0, 0);

  /** Creates a new FollowPathWithRotationSource. */
  public FollowPathWithRotationSource(Path path, Command command) {
    super(path);
    this.command = command;
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    command.schedule();
    // start running the command
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = PathingConstants.driveSubsystem.getPosition();
    PathState state = getPathState(robotPose);
    // The target relative to the robots current position
    Translation2d deltaLocation = state.goalPose.getTranslation().minus(robotPose.getTranslation());

    // Clamp state speed so the end of the path can be consistently reached
    // Clamped between [Const Max, 5 cm/s]
    double clampedSpeed = Clamp(
        state.speedMetersPerSecond, 
        PathingConstants.maxVelocityMeters, 
        0.5
    );

    AutoShuffleboardTab.distanceFromGoalEntry.setDouble(deltaLocation.getNorm() - lookAheadMeters);
    AutoShuffleboardTab.speedEntry.setDouble(clampedSpeed);
    AutoShuffleboardTab.lookAheadEntry.setDouble(lookAheadMeters);

    // Scale to goal speed. Speed input is in meters per second, while drive accepts normal values.
    Translation2d axisSpeeds = new Translation2d(clampedSpeed, deltaLocation.getAngle());

    // Set lookahead based upon speed of next point
    lookAheadMeters = Clamp(
        PathingConstants.lookAheadScalar * clampedSpeed,
        1, 0.2
    );

    double omegaRadPerSecond = -rotationController.calculate(
        robotPose.getRotation().getRadians(), 
        state.goalPose.getRotation().getRadians()
    );
    if (command instanceof AutoRotationSource) {
      Optional<Rotation2d> optionalGoal = ((AutoRotationSource)command).getGoalRotation();
      if (optionalGoal.isPresent()) {
        omegaRadPerSecond = -rotationController.calculate(
          optionalGoal.get().getRadians(), 
          0
        );
      }
    }

    // Construct chassis speeds from state values
    // Convert field oriented to robot oriented
    // Construct chassis speeds from state values
    // Convert field oriented to robot oriented
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        // Field oriented chassisSpeeds
        new ChassisSpeeds(
            axisSpeeds.getX(),
            axisSpeeds.getY(),
            omegaRadPerSecond
        ), 
        // Rotate from current direction
        robotPose.getRotation()
    );
    // Drive
    PathingConstants.driveSubsystem.drive(speeds);

    // Recurse until called to end
  }

  // Returns true when the command should end.
  @Override
  /**
   * @return true when the robot is within the last point tolerance
   * and has passed the second to last point.
   */
  public boolean isFinished() {
    System.out.println("Is finished - " + command.isFinished());
    return command.isFinished();
  }
}
