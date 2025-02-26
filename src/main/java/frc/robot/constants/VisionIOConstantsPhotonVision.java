// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

public class VisionIOConstantsPhotonVision implements VisionIOConstants {

  public final String cameraName;
  public final Transform3d robotToCamera;
  public final Pipeline [] pipelines;

  public enum Pipeline {
    REFLECTIVE,
    COLORED_SHAPE,
    APRILTAG
  }
  
  public VisionIOConstantsPhotonVision(String cameraName, Transform3d robotToCamera, Pipeline[] pipelines) {
    this.cameraName = cameraName;
    this.robotToCamera = robotToCamera;
    this.pipelines = pipelines;
  }

  public VisionIOPhotonVision init() {
    return new VisionIOPhotonVision(cameraName, robotToCamera, pipelines);
  }
}