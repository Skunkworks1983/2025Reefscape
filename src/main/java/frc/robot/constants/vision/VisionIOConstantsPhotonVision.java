// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;


public class VisionIOConstantsPhotonVision implements VisionIOConstants {
    public String cameraName;
    public Transform3d robotToCamera;

    public VisionIOConstantsPhotonVision(String cameraName, Transform3d robotToCamera) {
        this.cameraName = cameraName;
        this.robotToCamera = robotToCamera;
    }

    @Override
    public Optional<VisionIO> initialize() {
        try {
            VisionIO io = new VisionIOPhotonVision(this.cameraName, this.robotToCamera);
            return Optional.of(io);
        } catch(Exception e) {
            System.err.println("Unable to initialize " + cameraName + " camera.");
            return Optional.empty();
        }
    }
}
