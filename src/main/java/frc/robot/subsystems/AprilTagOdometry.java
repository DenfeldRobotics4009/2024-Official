// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.auto.util.Field;

/**
 * This class will manage processes related to photon vision,
 * and holds helper functions to call relevant data based upon
 * the stored camera instance.
 * 
 * Each instance of this class will manage a single camera 
 */
public class AprilTagOdometry extends SubsystemBase {

    public static AprilTagFieldLayout aprilTagFieldLayout;

    PhotonPoseEstimator photonPoseEstimator;
    public PhotonCamera camera;

    final int speakerID;

    /**
     * Creates an april tag odometry source relating to a 
     * camera with the specified name, with the specified
     * position (in meters) from the center of the robot.
     * 
     * This class requires accurate calibration of the
     * photonvision system, as described at:
     * 
     * https://docs.photonvision.org/en/latest/docs/calibration/calibration.html
     * 
     * @param cameraName
     * @param cameraToRobot offset in meters from the robot to the camera
     */
    public AprilTagOdometry(PhotonCamera camera, Transform3d robotToCamera) {

        this.camera = camera;
        try {

            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
                AprilTagFields.k2024Crescendo.m_resourceFile
            );

            photonPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.AprilTagOdometry.cameraPose);

        } catch (IOException e) {
            // Id be quite surprised if this is an error, handle nonetheless
            DriverStation.reportWarning(
                "April tag data could not be loaded, april tag positioning will not occur.", false);

            e.printStackTrace();
        }

        // Grab speaker id based on alliance position
        if (Field.isRedAlliance()) {
            speakerID = 4;
        } else {
            speakerID = 7;
        }

    }

    public List<PhotonTrackedTarget> getTargets() {
        return getPipelineResult().targets;
    }

    /**
     * Returns the target of a given id, if that target
     * is currently visible by this camera.
     * @param id april tag id
     * @return
     */
    public Optional<PhotonTrackedTarget> getTarget(int id) {
        for (PhotonTrackedTarget target : getTargets()) {
            if (target.getFiducialId() == id) return Optional.of(target);
        }
        return Optional.empty();
    }

    public Optional<PhotonTrackedTarget> getBestTarget() {
        PhotonPipelineResult result = getPipelineResult();
        if (!result.hasTargets()) return Optional.empty();
        return Optional.ofNullable(getPipelineResult().getBestTarget());
    }

    public PhotonPipelineResult getPipelineResult() {
        return camera.getLatestResult();
    }

    public Optional<Pose3d> getTargetPose(PhotonTrackedTarget target) {
        int fiducialId = target.getFiducialId();
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(fiducialId);
        // Ensure the existence of this tag id, print warning
        if (tagPose.isEmpty()) {
            DriverStation.reportWarning("Fiducial id " + fiducialId + " not recognized", false);
        }
        // Return with optional null value
        return tagPose;
    }

    /**
     * Calculates and returns the best position from relevant values
     */
    public Optional<Pose2d> getPosition() {
        // Check for 0
        return getPositionFromTargets();
    }

    
    public double getDistanceToTarget(PhotonTrackedTarget target) {
        Optional<Pose3d> tagPose = getTargetPose(target);
        // Ensure the existence of this tag id
        if (tagPose.isEmpty()) {return -1;}

        Transform3d cameraToTarget = target.getBestCameraToTarget();
        return Math.hypot(cameraToTarget.getX(), cameraToTarget.getY());
    }

    /**
     * @return Distance in meters
     */
    public double getDistanceToSpeaker() {

        Optional<PhotonTrackedTarget> target = getTarget(speakerID);
    
        if (target.isPresent()) return getDistanceToTarget(target.get());

        Translation2d targetPose = AprilTagOdometry.aprilTagFieldLayout.getTagPose(speakerID).get().getTranslation().toTranslation2d();
        
        if (Field.isRedAlliance()) {
            targetPose = Field.translateRobotPoseToRed(targetPose);
        }

        return SwerveDrive.getInstance().getPosition().getTranslation().getDistance(targetPose);
    }

    /**
     * @return Degrees, relative to field
     */
    public double getYawToSpeaker() {
        Optional<PhotonTrackedTarget> target = getTarget(speakerID);
        if (target.isPresent()) return -target.get().getYaw();


        Translation2d targetPose = AprilTagOdometry.aprilTagFieldLayout.getTagPose(speakerID).get().getTranslation().toTranslation2d();
        if (Field.isRedAlliance()) {
            targetPose = Field.translateRobotPoseToRed(targetPose);
        }
        System.out.println(targetPose);

        return targetPose.minus(
            SwerveDrive.getInstance().getPosition().getTranslation()
        ).getAngle().minus(SwerveDrive.getInstance().getPosition().getRotation()).getDegrees();
    }

    Optional<Pose2d> getPositionFromTargets() {
        // Catch if the layout was not able to be initialized
        if (aprilTagFieldLayout == null) {
            return Optional.empty();
        }
        
        Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update();
        // Break if pose cannot be calculated
        if (estimatedPose.isEmpty()) {
            return Optional.empty();
        }

        Pose2d estimatedPose2d = estimatedPose.get().estimatedPose.toPose2d();
        // Rotate if were on the red team
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            estimatedPose2d = Field.translateRobotPoseToRed(estimatedPose2d);
        }
        return Optional.of(estimatedPose2d);
    }

    @Override
    public void periodic() {
        // If we have a target, add the vision requirement to the drivetrain
        Optional<Pose2d> positionSample = getPositionFromTargets();
        if (positionSample.isPresent()) {
            SwerveDrive.getInstance().addVisionMeasurement(
                positionSample.get(), Timer.getFPGATimestamp()
            );
        }

        SmartDashboard.putNumber("Yaw to speaker", getYawToSpeaker());
        SmartDashboard.putNumber("Dist to speaker", getDistanceToSpeaker());
    }
}
