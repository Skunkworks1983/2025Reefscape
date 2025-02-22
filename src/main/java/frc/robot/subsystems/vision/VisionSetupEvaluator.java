// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionSetupEvaluator {

  public class PoseWrapper {
    public double x;
    public double y;
    public double rot; // Degrees

    public PoseWrapper(double x, double y, double rot) {
      this.x = x;
      this.y = y;
      this.rot = rot;
    }
  }

  // The basic list
  private final double rollingAvgLength = 10;
  private List<Pose2d> poses = new LinkedList<Pose2d>();
  private double xDeviationAvg = 0, yDeviationAvg = 0, rotDeviationAvg = 0;

  public VisionSetupEvaluator() {}

  /** Update the rolling average of Pose2d's with a new measurement */
  public PoseWrapper updateAvgDeviations(Pose2d newMeasurement) {

    while(poses.size() >= rollingAvgLength) {
      poses.remove(0);
    }

    poses.add(newMeasurement);

    double xDeviation = 0, yDeviation = 0, rotDeviation = 0;

    for(Pose2d i : poses) {
      xDeviation += i.getX();
      yDeviation += i.getY();
      rotDeviation += i.getRotation().getDegrees();
    }

    xDeviationAvg = xDeviation / rollingAvgLength;
    yDeviationAvg = yDeviation / rollingAvgLength;
    rotDeviationAvg = rotDeviation / rollingAvgLength;

    return new PoseWrapper(xDeviationAvg, yDeviationAvg, rotDeviationAvg);
  }
}
