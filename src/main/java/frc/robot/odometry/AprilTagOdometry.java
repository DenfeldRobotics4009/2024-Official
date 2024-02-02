// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.odometry;

import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;

import org.photonvision.PhotonCamera;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.auto.util.Field;

/**
 * This class will manage processes related to photon vision,
 * and holds helper functions to call relevant data based upon
 * the stored camera instance.
 * 
 * Each instance of this class will manage a single camera 
 */
public class AprilTagOdometry extends SubsystemBase implements OdometrySource {
    
    protected static class positionSample {
        final Pose2d sample;
        final double time;

        public positionSample(Pose2d sample, double time) {
            this.sample = sample;
            this.time = time;
        }
    }

    public final PhotonCamera camera;
    public final Transform3d cameraToRobot;

    AprilTagFieldLayout aprilTagFieldLayout;
    Pose3d lastRobotPose = new Pose3d();

    LinkedList<positionSample> positionSamples = new LinkedList<positionSample>();
    Pose2d sampleSum = new Pose2d();

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
    public AprilTagOdometry(String cameraName, Transform3d cameraToRobot) {

        OdometryHandler.addSource(this, OdometryType.External);

        camera = new PhotonCamera(cameraName);
        this.cameraToRobot = cameraToRobot;

        try {

            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
                AprilTagFields.k2024Crescendo.m_resourceFile
            );

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
     * Calculates and returns the position of the robot via april-tags
     * currently viewable by the robots camera(s).
     * @returns Optional<Pose2d>, nothing if no april-tags are seen.
     */
    public Optional<Pose2d> getPosition() {
        // Pose2d position = null;
        // // Catch if the layout was not able to be initialized
        // if (aprilTagFieldLayout == null) {return Optional.ofNullable(position);}

        // Optional<PhotonTrackedTarget> bestTarget = getBestTarget();
        // if (bestTarget.isPresent()) {
        //     PhotonTrackedTarget target = bestTarget.get();
        //     Optional<Pose3d> tagPose = getTargetPose(target);
        //     // Ensure the existence of this tag id
        //     if (tagPose.isEmpty()) {return Optional.ofNullable(position);}
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

    
    public double getDistanceToTarget(PhotonTrackedTarget target) {
        Optional<Pose3d> tagPose = getTargetPose(target);
        // Ensure the existence of this tag id
        if (tagPose.isEmpty()) {return -1;}

        return PhotonUtils.calculateDistanceToTargetMeters(
                cameraToRobot.getZ(), 
                tagPose.get().getZ(), 
                cameraToRobot.getRotation().getZ(), 
                target.getPitch()
            );
    }

    @Override
    public void setPosition(Pose2d position) {
        // April tag positioning cannot be corrected
    }

    Optional<Pose2d> getPositionFromTargets() {
        Pose2d position = null;
        // Catch if the layout was not able to be initialized
        if (aprilTagFieldLayout == null) {return Optional.ofNullable(position);}
        Optional<PhotonTrackedTarget> bestTarget = getBestTarget();
        if (bestTarget.isPresent()) {
            PhotonTrackedTarget target = bestTarget.get();
            Optional<Pose3d> tagPose = getTargetPose(target);
            // Ensure the existence of this tag id
            if (tagPose.isEmpty()) {return Optional.ofNullable(position);}
            // Assign robot position
            position = PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(), tagPose.get(), cameraToRobot).toPose2d();

            if (DriverStation.getAlliance().get() == Alliance.Red) {
                // Flip the assumed robot position if we are on the red alliance
                position = new Pose2d(
                    Field.translateRobotPoseToRed(position.getTranslation()), 
                    // Rotate angle by 180
                    position.getRotation().plus(new Rotation2d(Math.PI))
                );
            }
        }

        return Optional.ofNullable(position);
    }


    @Override
    public void periodic() {
        Optional<Pose2d> currentPose = getPositionFromTargets();
        if (currentPose.isPresent()) {

            Pose2d average = sampleSum.div(positionSamples.size());

            positionSamples.addLast(
                new positionSample(currentPose.get(), System.currentTimeMillis())
            );

            Transform2d transform = new Transform2d(new Pose2d(), currentPose.get());
            sampleSum = sampleSum.plus(transform);


        }

    }
}
