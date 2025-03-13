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

  // TODO: remove atomic reference as they are unnessecary
  public AtomicReference<Double> lidarDistanceRight = new AtomicReference<>();
  public AtomicReference<Double> lidarDistanceLeft = new AtomicReference<>();

  public BooleanSupplier isLidarRightTripped = () -> lidarDistanceRight.get() > Constants.Drivebase.LIDAR_RIGHT_TRIGGER_DISTANCE;
  public BooleanSupplier isLidarLeftTripped = () -> lidarDistanceLeft.get() > Constants.Drivebase.LIDAR_LEFT_TRIGGER_DISTANCE;

  private Thread thread = new Thread(this::updateDistance);

  public DualLidar() {
    lidarDistanceRight.set(0.0);
    lidarDistanceLeft.set(0.0);

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

  private void updateDistance() {

    while(true) {
      double startTime = Timer.getFPGATimestamp();
      outputLeft.set(true);
      outputRight.set(true);
      try {
        Thread.sleep(1);
      } catch (InterruptedException e) {
        e.printStackTrace();
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

      lidarDistanceRight.set(distanceRight);
      lidarDistanceLeft.set(distanceLeft);

      outputLeft.set(false);
      outputRight.set(false);

      double timeElapsed = Timer.getFPGATimestamp() - startTime;
      try {
        Thread.sleep((long)Units.secondsToMilliseconds(Math.max(Constants.RoboRIOInfo.UPDATE_PERIOD - timeElapsed, 0.0)));
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }
}
