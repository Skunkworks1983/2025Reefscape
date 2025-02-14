package frc.robot.utils;

import java.util.Optional;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;

// Signal value represents a signal (essensially an advanced supplier for sensors), an optional 
// derivative signal (to estimate where the most recent value is based on the change in time from the
// most recent sample), and the most recent sampled value of the previous two values.
public class SignalValue
    <U extends Unit>
 {
  private StatusSignal<? extends Measure<U>> statusSignal;
  // This optional value represents the derivative of 
  private Optional<StatusSignal<? extends Measure<PerUnit<U,TimeUnit>>>> statusSignalSlope;

  // This represents the most recent _cached_ result of the signals stored in this class.
  private double value;

  public SignalValue(
    StatusSignal<? extends Measure<U>> statusSignal,
    Optional<StatusSignal<? extends Measure<PerUnit<U,TimeUnit>>>> statusSignalSlope, 
    double value
  ) {
    this.statusSignal = statusSignal;
    this.statusSignalSlope = statusSignalSlope;
    this.value = value;
  }

  public StatusSignal<? extends Measure<U>> getStatusSignal() {
    return statusSignal;
  }

  public Optional<StatusSignal<? extends Measure<PerUnit<U,TimeUnit>>>> getStatusSignalSlope() {
    return statusSignalSlope;
  }

  public double getValue() {
    return value;
  }

  public void setValue(double value) {
    this.value = value;
  }
}
