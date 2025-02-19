// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemSignals;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.stream.Stream;

import com.ctre.phoenix6.BaseStatusSignal;

import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.SignalValue;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState.PhoenixSubsystemState;

/** 
 * This class represents all the SignalValues of a subsystem. This class contains methods
 * to add SignalValues (mapped to the FIELD datatype), view the BaseStatusSignals, update
 * values stored within the SignalValues (<code>updateCachedValues<code>), and functions
 * to lock and unlock resources connected to this class. On construction, this class
 * takes a FIELD, a type to use for hashing of the signalValues. The FIELD will generally be
 * an enum with names like TURN_MOTOR_POSITION, DRIVE_MOTOR_VELOCITY. 
 */
public abstract class SubsystemSignal<FIELD> {
  public ReentrantReadWriteLock stateLock = new ReentrantReadWriteLock();

  public Map<FIELD, SignalValue> signalValueMap = new HashMap<>();

  public List<BaseStatusSignal> getSignals() {
    List<BaseStatusSignal> baseStatusSignals = signalValueMap.values()
      .stream()
      .map(signalValue -> signalValue.getSignal())
      .toList();

    List<BaseStatusSignal> baseStatusSignalRates = signalValueMap.values()
      .stream()
      .filter(signalValue -> signalValue.getSignalSlope().isPresent())
      .map(signalValue -> signalValue.getSignalSlope().get())
      .toList();

    return Stream.concat(baseStatusSignals.stream(), baseStatusSignalRates.stream())
      .toList();
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


  public abstract PhoenixSubsystemState<FIELD> getState();
}
