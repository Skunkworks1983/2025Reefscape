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
import frc.robot.constants.vision.VisionConstants;
import frc.robot.constants.vision.VisionIOConstants;
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
  private LinkedList<VisionIO> io = new LinkedList<VisionIO>();
  private List<Field2d> field2ds = new LinkedList<Field2d>();
  private final AprilTagFieldLayout aprilTagLayout = 
    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  
  public Vision(VisionConsumer consumer, VisionIOConstants... ioConstants) {
      this.consumer = consumer;
  
      for (VisionIOConstants i : ioConstants) {
        try {
          VisionIO inited = i.init();
          io.add(inited);
          Field2d field = new Field2d();
          SmartDashboard.putData(inited.getName() + " Odometry", field);
          field2ds.add(field);
        } catch (Exception e) {
          System.err.println("A Vision IO failed to initialize");
          e.printStackTrace();
          continue;
        }      
      }
    }
  
    @Override
    public void periodic() {
  
      for (int i = 0; i < io.size(); i++) {
        VisionIOData data = io.get(i).getLatestData();
        for (PoseObservation observation : data.poseObservations) {

          field2ds.get(i).setRobotPose(observation.estimatedPose().toPose2d());
  
          ConditionalSmartDashboard.putNumber(io.get(i).getName() + " Latest Ambiguity", observation.ambiguity());
          ConditionalSmartDashboard.putNumber(io.get(i).getName() + " Latest Z Error", observation.estimatedPose().getZ());
          ConditionalSmartDashboard.putNumber(io.get(i).getName() + " Average Tag Distance", observation.averageTagDistance());
  
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
          
          double x = observation.averageTagDistance();

          final double kA = 0.0329 + .005;
          final double kB = -0.0222 + .005;
          final double kC = 0.0048 + .005; // Adding a small value to constant term to decrease overall confidence

          // This is the
          double linearStdDev = (kA * Math.pow(x, 2)) + (kB * x) + kC;

          ConditionalSmartDashboard.putNumber(io.get(i).getName() + " Linear Std Dev", linearStdDev);
  
          consumer.accept(
              observation.estimatedPose().toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, VisionConstants.ANGULAR_STD_DEV));
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
