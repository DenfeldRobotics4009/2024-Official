// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.*;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * This class will manage processes related to photon vision,
 * and holds helper functions to call relevant data based upon
 * the stored camera instance.
 * 
 * Each instance of this class will manage a single camera 
 */
public class AprilTagOdometry {

    public final PhotonCamera camera;
    public final Transform3d cameraToRobot;

    AprilTagFieldLayout aprilTagFieldLayout;
    Pose3d lastRobotPose;

    /**
     * Records the amount of time that has passed since the
     * last record of the robots position.
     */
    Timer lastRecordedPose = new Timer();

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

        camera = new PhotonCamera(cameraName);
        this.cameraToRobot = cameraToRobot;

        lastRecordedPose.start();

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

    /*
     * This method should be called periodically to keep the last
     * recorded robot position accurate.
     */
    public void updateRobotPose() {
        // Catch if the layout was not able to be initialized
        if (aprilTagFieldLayout == null) {return;}

        PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
        // This function may be called without a valid target
        if (target != null) {
            int fiducialId = target.getFiducialId();
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(fiducialId);
            // Ensure the existence of this tag id
            if (tagPose.isEmpty()) {
                DriverStation.reportWarning("Fiducial id " + fiducialId + " not recognized", false);
                return;
            }
            // Assign robot position
            lastRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(), tagPose.get(), cameraToRobot);

            // Reset timer
            lastRecordedPose.reset();
        }
    }

    /**
     * Gets the most recently updated robot position
     * @return pose3d of position. Ideally, the Z axis
     * of the pose should remain zero, unless the robot
     * is not on the carpet.
     */
    public Pose3d getRobotPose() {
        updateRobotPose();
        return lastRobotPose;
    }

    /**
     * @return seconds sinze the robot pose has been updated
     */
    public double getTimeSinceLastRecording() {
        return lastRecordedPose.get();
    }

}
