// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOData;

/**
 * A subsystem that takes a <code>VisionConsumer</code> and any number of
 * <code>VisionIO</code>. Calls
 * <code>accept()</code> on the VisionConsumer in <code>periodic()</code> to
 * send latest vision measurements. <code>VisionConsumer</code> intended to
 * be used by a <code>SwerveDrivePoseEstimator</code> in <code>Drivebase</code>.
 */
public class Vision extends SubsystemBase {

  VisionConsumer consumer;
  VisionIO[] io;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;
  }

  @Override
  public void periodic() {
    for (VisionIO io : io) {
      VisionIOData data = io.getLatestData();
      for (PoseObservation observation : data.poseObservations) {
        consumer.accept(observation.estimatedPose(), observation.timestamp(), observation.stdDevs());
      }
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d estimatedPose,
        double timestamp,
        Matrix<N3, N1> stdDevs);
  }
}
