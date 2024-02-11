// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NoteCamera extends SubsystemBase {

  final PhotonCamera camera;

  /** Creates a new NoteCamera. */
  public NoteCamera(PhotonCamera camera) {
    this.camera = camera;
    //camera.setPipelineIndex(Constants.NoteDetection.noteDetectionPipelineIndex);
  }

  /**
   * @return A list of all present notes seen
   */
  public List<PhotonTrackedTarget> getTargets() {
    return camera.getLatestResult().getTargets();
  }
  
  /**
   * @return a double representing the angle offset from the
   * camera to the note. If the value is positive, the note
   * is to the left of the camera. If no note is present,
   * the optional will be empty.
   */
  public Optional<Double> getYawToNote() {
    PhotonPipelineResult result = camera.getLatestResult();

    if (!result.hasTargets()) {
      return Optional.empty(); 
    }

    return Optional.ofNullable(result.getBestTarget().getYaw());
  }

  @Override
  public void periodic() {

  }
}
