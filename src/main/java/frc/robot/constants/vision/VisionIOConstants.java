// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.vision;

import java.util.Optional;

import frc.robot.subsystems.vision.VisionIO;

/** A factory class for initializing a VisionIO. */
public interface VisionIOConstants {

    /**
     * Construct and return a VisionIO implementation. If unable to
     * construct it (for instance, the camera is not connected), return
     * Optional.empty().
     */
    public Optional<VisionIO> initialize();
}
