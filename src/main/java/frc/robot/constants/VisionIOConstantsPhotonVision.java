// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

public class VisionIOConstantsPhotonVision implements VisionIOConstants {

  public final String cameraName;
  public final Transform3d robotToCamera;
  public final PhotonVisionPipelineType [] pipelines;

  public enum PhotonVisionPipelineType {
    REFLECTIVE,
    COLORED_SHAPE,
    APRILTAG
  }

  /**
   * @param cameraName
   * @param robotToCamera
   * @param pipeline
   * @param additionalPipelines
   */
  public VisionIOConstantsPhotonVision(String cameraName, Transform3d robotToCamera, PhotonVisionPipelineType pipeline, PhotonVisionPipelineType... additionalPipelines) {
    this.cameraName = cameraName;
    this.robotToCamera = robotToCamera;

    // Initialize pipeline array
    this.pipelines = new PhotonVisionPipelineType[1+additionalPipelines.length];
    this.pipelines[0] =  pipeline;
    for(int i = 0; i<additionalPipelines.length; i++) {
      this.pipelines[i+1] = additionalPipelines[i];
    }
  }

  public VisionIOPhotonVision init() {
    return new VisionIOPhotonVision(cameraName, robotToCamera, pipelines);
  }
}