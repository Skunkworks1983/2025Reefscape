// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants.VisionConstants;

/**
 * Use for keeping a rolling average of standard deviations in x, y, and rotation for vision pose estimations.
 */
public class PoseDeviations {

  /**
   * A wrapper class for a Pose2d. Using this rather than a 
   * Pose2d because the Rotation2d may not be continuous.
   */
  public class PoseWrapper {
    public final double x, y, rot; // Rotation units is degrees

    public PoseWrapper(double x, double y, double rot) {
      this.x = x;
      this.y = y;
      this.rot = rot;
    }
  }

  
  // The rolling average list of pose measurements.
  private List<PoseWrapper> data = new LinkedList<PoseWrapper>();

  // The maximum length of the rolling average list.
  private static final int rollingAvgLength = VisionConstants.JITTER_TEST_ROLLING_AVG_LENGTH;
  
  private Sendable stdDevsSendable = new Sendable() {
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("X Std Dev", () -> calculateStdDevs().x, null);
      builder.addDoubleProperty("Y Std Dev", () -> calculateStdDevs().y, null);
      builder.addDoubleProperty("Rot Std Dev", () -> calculateStdDevs().rot, null);
    }
  };

  public PoseDeviations() {
    SmartDashboard.putData(stdDevsSendable);
  }

  /** 
   * Update the rolling average with a new pose measurement. 
   * The most out-of-date measurement gets dropped from the front of the list, 
   * and the new measurement is appended to the end. Intended to be called 
   * once per loop in {@link Vision}'s periodic().
   */
  public void updateMeasurements(Pose2d newMeasurement) {

    while(data.size() >= rollingAvgLength) {
      data.remove(0);
    }

    data.add(
      new PoseWrapper(
        newMeasurement.getX(), 
        newMeasurement.getY(),
        newMeasurement.getRotation().getDegrees()
      )
    );
  }

  public PoseWrapper calculateStdDevs() {
    /*  
      Using immutable variables so that a 
      weird multithreading case can't break this. 
    */
    final List<PoseWrapper> kData = data;
    final int kDataLength = kData.size();
    double xSum = 0, ySum = 0, rotSum = 0;

    // Sum x, y, and rotation
    for(PoseWrapper pose : kData) {
      xSum += pose.x;
      ySum += pose.y;
      rotSum += pose.rot;
    }

    // Calculate averages
    double xMean = xSum / kDataLength;
    double yMean = ySum / kDataLength;
    double rotMean = rotSum / kDataLength;

    double sumXDeviations = 0, sumYDeviations = 0, sumRotDeviations = 0;
    
    // Sum squared deviations
    for(PoseWrapper pose : kData) {
      sumXDeviations += Math.pow((pose.x - xMean), 2);
      sumYDeviations += Math.pow((pose.y - yMean), 2);
      sumRotDeviations += Math.pow((pose.rot - rotMean), 2);
    }

    // Calculate standard deviations
    double xStdDev = Math.sqrt(sumXDeviations / kDataLength);
    double yStdDev = Math.sqrt(sumYDeviations / kDataLength);
    double rotStdDev = Math.sqrt(sumRotDeviations / kDataLength);

    return new PoseWrapper(xStdDev, yStdDev, rotStdDev);
  }
}
