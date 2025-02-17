// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.odometry.subsystemSignals;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.BaseStatusSignal;

import frc.robot.utils.odometry.SignalValue;
import frc.robot.utils.odometry.subsystemState.SubsystemState;

/** Add your docs here. */
public abstract class SubsystemSignal<FIELD> {

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

  protected void addField(FIELD key, SignalValue value) {
    signalValueMap.put(key, value);
  }


  public abstract SubsystemState<FIELD> getState();
}
