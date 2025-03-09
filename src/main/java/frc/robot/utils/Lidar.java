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
  private boolean isOn;
  private double dist;
  private double triggerCutoff;

  private Counter lidar;
  private DigitalOutput output;

  public Lidar(int dataPort, int triggerPort, double triggerDistance, double triggerCutoff) {
    this.triggerDistance = triggerDistance;
    this.triggerCutoff = triggerCutoff;
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
    if(currentTime - lastTime < 0.04) {
      return dist;
    } 
    output.set(true);
    lastTime = currentTime;
    while(Timer.getFPGATimestamp() - lastTime < 0.005) {

    }
    double v = lidar.getPeriod();
    double d;
    if(lidar.get() < 1){
      d = 0;
    }
    else {
      d = v * 1000000.0 / 10.0;
    }
    if(d > triggerCutoff) {
      d = dist;
    }
    dist = d;
    output.set(false);
    return dist;
  }
}
