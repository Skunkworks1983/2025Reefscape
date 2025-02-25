// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOData;
import frc.robot.utils.ConditionalSmartDashboard;

/**
 * A subsystem that takes a VisionConsumer and any number of
 * VisionIOConstants. Calls
 * accept() on the VisionConsumer in periodic() to
 * send latest vision measurements. VisionConsumer is intended to
 * be used by a SwerveDrivePoseEstimator in Drivebase.
 */
public class Vision extends SubsystemBase {

  private VisionConsumer consumer;
  private VisionIO[] io;
  private List<Field2d> field2ds = new LinkedList<Field2d>();
  private final AprilTagFieldLayout aprilTagLayout = 
    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    for (VisionIO i : io) {
      Field2d field = new Field2d();
      SmartDashboard.putData(i.getName() + " Odometry", field);
      field2ds.add(field);
    }
  }

  @Override
  public void periodic() {

    for (int i = 0; i < io.length; i++) {
      VisionIOData data = io[i].getLatestData();
      for (PoseObservation observation : data.poseObservations) {

        ConditionalSmartDashboard.putNumber(io[i].getName() + " Latest Ambiguity", observation.ambiguity());
        ConditionalSmartDashboard.putNumber(io[i].getName() + " Latest Z Error", observation.estimatedPose().getZ());
        ConditionalSmartDashboard.putNumber(io[i].getName() + " Average Tag Distance", observation.averageTagDistance());

        boolean rejectPose = 
          observation.tagCount() == 0 ||
          observation.ambiguity() > VisionConstants.MAX_AMBIGUITY ||
          observation.estimatedPose().getZ() > VisionConstants.MAX_Z_ERROR ||
          observation.estimatedPose().getX() < 0.0 ||
          observation.estimatedPose().getX() > aprilTagLayout.getFieldLength() ||
          observation.estimatedPose().getY() < 0.0 ||
          observation.estimatedPose().getY() > aprilTagLayout.getFieldWidth() ||
          observation.averageTagDistance() > VisionConstants.MAX_AVERAGE_TAG_DISTANCE;

        if (rejectPose) {
          continue;
        }

        double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = VisionConstants.LINEAR_STD_DEV_BASELINE * stdDevFactor;
        double angularStdDev = VisionConstants.ANGULAR_STD_DEV_BASELINE * stdDevFactor;

        consumer.accept(
            observation.estimatedPose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

        field2ds.get(i).setRobotPose(observation.estimatedPose().toPose2d());
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
