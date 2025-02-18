// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemSignals;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.BaseStatusSignal;

import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.SignalValue;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState.SubsystemState;

/** Add your docs here. */
public abstract class SubsystemSignal<FIELD> {
  public ReentrantReadWriteLock stateLock;

  public Map<FIELD, SignalValue> signalValueMap = new HashMap<>();

  public List<BaseStatusSignal> getSignals() {
    List<BaseStatusSignal> baseStatusSignals = signalValueMap.values()
      .stream()
      .map(signalValue -> signalValue.getSignal())
      .toList();

    baseStatusSignals.addAll(signalValueMap.values()
      .stream()
      .filter(signalValue -> signalValue.getSignalSlope().isPresent())
      .map(signalValue -> signalValue.getSignalSlope().get())
      .toList()
    );

    return baseStatusSignals;
  }

  public void updateCachedValues() {
    for(FIELD key : signalValueMap.keySet()) {
      signalValueMap.get(key).updateCachedValue();
    }
  }

  public ReentrantReadWriteLock getLock() {
    return stateLock;
  }

  protected void addField(FIELD key, SignalValue value) {
    signalValueMap.put(key, value);
  }


  public abstract SubsystemState<FIELD> getState();
}
