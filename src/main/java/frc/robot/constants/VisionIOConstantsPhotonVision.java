// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

public class VisionIOConstantsPhotonVision implements VisionIOConstants {

  public String cameraName;
  public Transform3d robotToCamera;
   
  public VisionIOConstantsPhotonVision(String cameraName, Transform3d robotToCamera) {
    this.cameraName = cameraName;
    this.robotToCamera = robotToCamera;
  }

  public VisionIOPhotonVision initialize() {
    return new VisionIOPhotonVision(cameraName, robotToCamera);
  }
}
