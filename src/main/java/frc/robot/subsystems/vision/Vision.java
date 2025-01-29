// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final List<Field2d> field2ds = new LinkedList<Field2d>();

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    for (VisionIO i : io) {
      Field2d field = new Field2d();
      SmartDashboard.putData(i.getName() + " Visual Odometry", field);
      field2ds.add(field);
    }

    System.out.println("Vision Constructor Running");
  }

  @Override
  public void periodic() {
    System.out.println("Vision Periodic Running");
    for (int i = 0; i < io.length; i++) {
      VisionIOData data = io[i].getLatestData();
      for (PoseObservation observation : data.poseObservations) {
        consumer.accept(observation.estimatedPose(), observation.timestamp(), observation.stdDevs());
        // field2ds.get(i).setRobotPose(observation.estimatedPose());
      }
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d estimatedPose,
        double timestamp,
        Matrix<N3, N1> stdDevs
    );
  }
}
