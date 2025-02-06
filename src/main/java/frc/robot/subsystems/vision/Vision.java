// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.vision.VisionIOConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOData;

/**
 * A subsystem that takes a VisionConsumer and any number of
 * VisionIOConstants. Calls
 * accept() on the VisionConsumer in periodic() to
 * send latest vision measurements. VisionConsumer is intended to
 * be used by a SwerveDrivePoseEstimator in Drivebase.
 */
public class Vision extends SubsystemBase {

  VisionConsumer consumer;
  List<VisionIO> presentIO = new LinkedList<VisionIO>();
  List<Field2d> field2ds = new LinkedList<Field2d>();

  public Vision(VisionConsumer consumer, VisionIOConstants... visionConstants) {
    this.consumer = consumer;

    for(VisionIOConstants i : visionConstants) {
      Optional<VisionIO> initialized = i.initialize();
      if (initialized.isPresent()) {
        presentIO.add(initialized.get());
        Field2d field = new Field2d();
        SmartDashboard.putData(initialized.get().getName() + " Odometry", field);
        field2ds.add(field);
      }
    }
  }
  
  @Override
  public void periodic() {
    for (int i = 0; i < presentIO.size(); i++) {
      VisionIOData data = presentIO.get(i).getLatestData();
      for (PoseObservation observation : data.poseObservations) {
        consumer.accept(observation.estimatedPose(), observation.timestamp(), observation.stdDevs());
        field2ds.get(i).setRobotPose(observation.estimatedPose());
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
