// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;

/** Add your docs here. */
public class DualLidar {
  private Counter lidarRight;
  private Counter lidarLeft;
  private DigitalOutput outputLeft;
  private DigitalOutput outputRight;

  // NOTE: using objects because reference are always atomic (double changes /might/ not be for some systems).
  public Double lidarDistanceRight = 0.0;
  public Double lidarDistanceLeft = 0.0;

  public BooleanSupplier isLidarRightTripped = () -> lidarDistanceRight > Constants.Drivebase.LIDAR_RIGHT_TRIGGER_DISTANCE;
  public BooleanSupplier isLidarLeftTripped = () -> lidarDistanceLeft > Constants.Drivebase.LIDAR_LEFT_TRIGGER_DISTANCE;

  private Thread thread = new Thread(this::updateDistance);

  public DualLidar() {

    lidarRight = new Counter(Constants.Drivebase.LIDAR_RIGHT_DATA_PORT);
    lidarRight.setMaxPeriod(1.0);
    lidarRight.setSemiPeriodMode(true);
    lidarRight.reset();

    lidarLeft = new Counter(Constants.Drivebase.LIDAR_LEFT_DATA_PORT);
    lidarLeft.setMaxPeriod(1.0);
    lidarLeft.setSemiPeriodMode(true);
    lidarLeft.reset();

    outputLeft = new DigitalOutput(Constants.Drivebase.LIDAR_LEFT_TRIGGER_PORT);
    outputRight = new DigitalOutput(Constants.Drivebase.LIDAR_RIGHT_TRIGGER_PORT);
    thread.start();
  }

  private void pulseLidar() {
    double startTime = Timer.getFPGATimestamp();
      outputLeft.set(true);
      outputRight.set(true);
      while(Timer.getFPGATimestamp() - startTime < 0.001) {}
      outputLeft.set(false);
      outputRight.set(false);
      try {
        Thread.sleep(20);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
  }

  private void updateDistance() {
    pulseLidar();
    double lastTime = Timer.getFPGATimestamp();

    while(true) {
      double startTime = Timer.getFPGATimestamp();
      if(Timer.getFPGATimestamp() - lastTime > 0.25) {
        pulseLidar();
        lastTime = Timer.getFPGATimestamp();
      }
      double rightValue = lidarRight.getPeriod();
      double leftValue = lidarLeft.getPeriod();
      double distanceRight;
      double distanceLeft;

      if(lidarRight.get() < 1){
        distanceRight = 0;
      } else {
        distanceRight = rightValue * 1000000.0 / 10.0;
      }

      if(lidarLeft.get() < 1){
        distanceLeft = 0;
      } else {
        distanceLeft = leftValue * 1000000.0 / 10.0;
      }

      // Ignore returned values that are too large (because they are not real values)
      if(distanceRight < Constants.Drivebase.LIDAR_RIGHT_DATA_CUTOFF) lidarDistanceRight = distanceRight;
      if(distanceLeft < Constants.Drivebase.LIDAR_LEFT_DATA_CUTOFF) lidarDistanceLeft = distanceLeft;

      double timeElapsed = Timer.getFPGATimestamp() - startTime;
      try {
        Thread.sleep((long)Units.secondsToMilliseconds(
          Math.max(
            (Constants.RoboRIOInfo.UPDATE_PERIOD) - timeElapsed,
            0.0
          )
        ));
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }
}
