// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState;

import java.util.concurrent.locks.ReentrantReadWriteLock;

import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemSignals.SubsystemSignal;

/** Add your docs here. */
public abstract class SubsystemState<FIELD> {

  SubsystemSignal<FIELD> signalSubsystem;
  public SubsystemState(SubsystemSignal<FIELD> signalSubsystem) {
    this.signalSubsystem = signalSubsystem;
  }

  public double getValue(FIELD field) {
    return signalSubsystem.signalValueMap.get(field).get();
  }

  public ReentrantReadWriteLock.ReadLock getReadLock() {
    return signalSubsystem.stateLock.readLock();
  }
}
