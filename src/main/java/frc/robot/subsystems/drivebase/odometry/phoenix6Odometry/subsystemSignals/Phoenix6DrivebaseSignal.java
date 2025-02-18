// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemSignals;

import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.SignalValue;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState.Phoenix6DrivebaseState;
  
public class Phoenix6DrivebaseSignal extends SubsystemSignal
  <frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemSignals.Phoenix6DrivebaseSignal.Field> {

  public Phoenix6DrivebaseSignal(SignalValue gyroSignal) {
    super.signalValueMap.put(Field.GYRO, gyroSignal);
  }

  public static enum Field {
    GYRO
  }

  @Override
  public Phoenix6DrivebaseState getState() {
    return new Phoenix6DrivebaseState(this);
  }
}