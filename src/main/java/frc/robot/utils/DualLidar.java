// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;

/** Add your docs here. */
public class DualLidar {
  private double triggerDistance1;
  private double triggerDistance2;

  private Counter lidar1;
  private Counter lidar2;
  private DigitalOutput output;
  public AtomicReference<Double> lidarDistance1;
  public AtomicReference<Double> lidarDistance2;

  public BooleanSupplier isLidar1Tripped = () -> lidarDistance1.get() > triggerDistance1;
  public BooleanSupplier isLidar2Tripped = () -> lidarDistance2.get() > triggerDistance2;

  private Thread thread = new Thread(this::updateDistance);

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
    this.triggerDistance1 = triggerDistance2;

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

  private void updateDistance() {

    double thisTime = Timer.getFPGATimestamp();
    while(true) {
      output.set(true);
      try {
        Thread.sleep(1);
      } catch (InterruptedException e) {
        e.printStackTrace();
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

      lidarDistance1.set(d1);
      lidarDistance2.set(d2);

      output.set(false);

      double lastTime = thisTime;
      thisTime = Timer.getFPGATimestamp();
      double timeElapsed = thisTime - lastTime;
      try {
        Thread.sleep((long)Units.secondsToMilliseconds(Constants.PathPlanner.UPDATE_PERIOD - (timeElapsed)));
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }
}
