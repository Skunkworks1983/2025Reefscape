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
import frc.robot.constants.vision.VisionIOConstants;
import frc.robot.subsystems.vision.VisionIO.VisionMeasurement;

/**
 * A subsystem that takes a VisionConsumer and any number of
 * VisionIOConstants. Calls
 * accept() on the VisionConsumer in periodic() to
 * send latest vision measurements. VisionConsumer is intended to
 * be used by a SwerveDrivePoseEstimator in Drivebase.
 */
public class Vision extends SubsystemBase {

  private VisionConsumer consumer;
  private LinkedList<VisionIO> ios = new LinkedList<VisionIO>();
  private List<Field2d> field2ds = new LinkedList<Field2d>();
  
  public Vision(VisionConsumer consumer, VisionIOConstants... ioConstants) {
      this.consumer = consumer;
  
      for (VisionIOConstants constants : ioConstants) {
        try {
          VisionIO initializedIO = constants.init();
          ios.add(initializedIO);
          Field2d field2d = new Field2d();
          SmartDashboard.putData(initializedIO.getName() + " Odometry", field2d);
          field2ds.add(field2d);
        } catch (Exception e) {
          System.err.println( 
            "A vision i/o failed to initialize. Double-check that the camera is plugged in" +
            "and the camera is working."
          );
          e.printStackTrace();
          continue;
        }      
      }
    }
  
    @Override
    public void periodic() {
      for(VisionIO io : ios) {
        for(VisionMeasurement measurement : io.getUnreadMeasurements()) {
          consumer.accept(
            measurement.estimatedPose().toPose2d(),
            measurement.timestamp(),
            measurement.stdDevs()
          );
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
