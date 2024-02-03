// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.odometry;

import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.auto.util.Field;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class will manage processes related to photon vision,
 * and holds helper functions to call relevant data based upon
 * the stored camera instance.
 * 
 * Each instance of this class will manage a single camera 
 */
public class AprilTagOdometry extends SubsystemBase implements OdometrySource {

    public static class Camera {
        final PhotonCamera camera;
        final Transform3d robotToCamera;
        public Camera(PhotonCamera camera, Transform3d robotToCamera) {
            this.camera = camera;
            this.robotToCamera = robotToCamera;
        }
    }

    AprilTagFieldLayout aprilTagFieldLayout;

    PhotonPoseEstimator photonPoseEstimator;
    PhotonCamera camera;


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

        OdometryHandler.addSource(this, OdometryType.External);

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
    }

    public List<PhotonTrackedTarget> getTargets() {
        return getPipelineResult().targets;
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

    @Override
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

        System.out.println( "X: " +
         target.getBestCameraToTarget().getZ());
        System.out.println( "Z: " +
         target.getBestCameraToTarget().getX());

        Transform3d cameraToTarget = target.getBestCameraToTarget();
        return Math.hypot(cameraToTarget.getX(), cameraToTarget.getY());
    }

    @Override
    public void setPosition(Pose2d position) {
        // April tag positioning cannot be corrected
    }

    Optional<Pose2d> getPositionFromTargets() {
        // Catch if the layout was not able to be initialized
        if (aprilTagFieldLayout == null) {
            return Optional.empty();
        }
        if (
            Math.abs(SwerveDrive.getInstance().getVelocity().getTranslation().getNorm()) > 
            Constants.AprilTagOdometry.maxSpeed &&
            Math.abs(SwerveDrive.getInstance().getVelocity().getRotation().getRadians()) >
            Constants.AprilTagOdometry.maxRotation
        ) {
            return Optional.empty();
        }

        // Set reference to the relative positions
        // photonPoseEstimator.setReferencePose(
        //     OdometryHandler.processPose2dSet(
        //         OdometryHandler.getPose2ds(OdometryHandler.internal).toArray(new Pose2d[0])
        //     )
        // );
        
        Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update();
        // Break if pose cannot be calculated
        if (estimatedPose.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(estimatedPose.get().estimatedPose.toPose2d());

        // Optional<PhotonTrackedTarget> bestTarget = getBestTarget();
        // if (bestTarget.isPresent()) {
        //     SmartDashboard.putNumber("Distance", getDistanceToTarget(bestTarget.get()));

        //     if (getDistanceToTarget(bestTarget.get()) > Constants.AprilTagOdometry.maxDistance) {
        //         return Optional.empty();
        //     }

        //     PhotonTrackedTarget target = bestTarget.get();
        //     Optional<Pose3d> tagPose = getTargetPose(target);
        //     // Ensure the existence of this tag id
        //     if (tagPose.isEmpty()) {
        //         return Optional.ofNullable(position);
        //     }
        //     // Assign robot position
        //     position = PhotonUtils.estimateFieldToRobotAprilTag(
        //         target.getBestCameraToTarget(), tagPose.get(), cameraToRobot).toPose2d();

        //     if (DriverStation.getAlliance().get() == Alliance.Red) {
        //         // Flip the assumed robot position if we are on the red alliance
        //         position = new Pose2d(
        //             Field.translateRobotPoseToRed(position.getTranslation()), 
        //             // Rotate angle by 180
        //             position.getRotation().plus(new Rotation2d(Math.PI))
        //         );
        //     }
        // }

        // return Optional.ofNullable(position);
    }

    @Override
    public void periodic() {

    }
}
