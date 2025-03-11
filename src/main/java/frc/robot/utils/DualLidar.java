// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class DualLidar {
  private double lastTime;
  private double triggerDistance1;
  private double dist1;
  private double triggerCutoff1;
  private double triggerDistance2;
  private double dist2;
  private double triggerCutoff2;

  private Counter lidar1;
  private Counter lidar2;
  private DigitalOutput output;

  public DualLidar(
    int dataPort1, 
    double triggerDistance1, 
    double triggerCutoff1, 
    int dataPort2, 
    double triggerDistance2,
    double triggerCutoff2,
    int triggerPort
  ) {
    this.triggerDistance1 = triggerDistance1;
    this.triggerCutoff1 = triggerCutoff1;
    this.triggerDistance1 = triggerDistance2;
    this.triggerCutoff1 = triggerCutoff2;
    dist1 = 0;
    dist2 = 0;
    lastTime = 0;

    lidar1 = new Counter(dataPort1);
    lidar1.setMaxPeriod(1.0);
    lidar1.setSemiPeriodMode(true);
    lidar1.reset();

    lidar2 = new Counter(dataPort2);
    lidar2.setMaxPeriod(1.0);
    lidar2.setSemiPeriodMode(true);
    lidar2.reset();

    output = new DigitalOutput(triggerPort);
  }

  public boolean isLidar1Tripped() {
    return getLidar1Distance() > triggerDistance1;
  }

  public boolean isLidar2Tripped() {
    return getLidar2Distance() > triggerDistance2;
  }

  public double getLidar1Distance() {
    if(Timer.getFPGATimestamp() - lastTime >= 0.04) {
      lastTime = Timer.getFPGATimestamp();
      updateDistance();
    } 

    return dist1;
  }

  public double getLidar2Distance() {
    if(Timer.getFPGATimestamp() - lastTime >= 0.04) {
      lastTime = Timer.getFPGATimestamp();
      updateDistance();
    } 

    return dist2;
  }

  private void updateDistance() {
    output.set(true);
    while(Timer.getFPGATimestamp() - lastTime < 0.001) {

    }
    double v1 = lidar1.getPeriod();
    double v2 = lidar1.getPeriod();
    double d1;
    double d2;

    if(lidar1.get() < 1){
      d1 = 0;
    }
    else {
      d1 = v1 * 1000000.0 / 10.0;
    }

    if(lidar2.get() < 1){
      d2 = 0;
    }
    else {
      d2 = v2 * 1000000.0 / 10.0;
    }

    if(d1 > triggerCutoff1) {
      d1 = dist1;
    }

    if(d2 > triggerCutoff2) {
      d2 = dist2;
    }

    dist1 = d1;
    dist2 = d2;
    output.set(false);
  }
}
