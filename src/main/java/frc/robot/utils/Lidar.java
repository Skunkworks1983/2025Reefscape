// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class Lidar {
  private double triggerDistance;
  private double lastTime;
  private double dist;

  private Counter lidar;
  private DigitalOutput output;

  public Lidar(int dataPort, int triggerPort, double triggerDistance) {
    this.triggerDistance = triggerDistance;
    lidar = new Counter(dataPort);
    output = new DigitalOutput(triggerPort);

    lidar.setMaxPeriod(1.0);
    lidar.setSemiPeriodMode(true);
    lidar.reset();
    dist = 0;
    lastTime = 0;
  }

  public boolean isTripped() {
    return getDistance() > triggerDistance;
  }

  public double getDistance() {
    double currentTime = Timer.getFPGATimestamp();
    if(currentTime - lastTime < 0.01) {
      return dist;
    }
    lastTime = currentTime;
    output.set(true);
    double v = lidar.getPeriod();
    if(lidar.get() < 1){
      dist = 0;
    }
    else {
      dist = v * 1000000.0 / 10.0;
    }
    output.set(false);
    return dist;
  }
}
