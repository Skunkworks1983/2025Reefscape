// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase.odometry;

import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.Phoenix6Odometry;
import frc.robot.subsystems.drivebase.odometry.positionEstimation.PositionEstimator;

/** Add your docs here. */
public class OdometryThread {
private Phoenix6Odometry pheonix6Odometry;
private PositionEstimator positionEstimator;
  public OdometryThread(
    Phoenix6Odometry pheonix6Odometry, 
    PositionEstimator positionEstimator
  ) {
    this.pheonix6Odometry = pheonix6Odometry;
    this.positionEstimator = positionEstimator;
  }

  public void startThread() {
    update(); // update once so that states can never be null.
    thread.start();
  }

  Thread thread = new Thread(() -> { while(true) { update(); } });

  private void update() {
    pheonix6Odometry.update();
    positionEstimator.update();
  }
}
