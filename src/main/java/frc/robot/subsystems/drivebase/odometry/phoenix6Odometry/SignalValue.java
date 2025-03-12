package frc.robot.subsystems.drivebase.odometry.phoenix6Odometry;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Signal value represents a signal (essensially an advanced supplier for sensors), an optional 
 * derivative signal (to estimate where the most recent value is based on the change in time from the
 * most recent sample), and the most recent sampled value of the previous two values. SignalValue
 * is used to store the cached values of the signals in an orderly manner.
 */
public class SignalValue {
  final BaseStatusSignal signal;
  final Optional<BaseStatusSignal> signalSlope;
  double cachedValue;
  double conversionFactor; // cachedValue * conversionFactor = the output value

  public SignalValue(BaseStatusSignal signal, double conversionFactor) {
    this.signal = signal;
    this.signalSlope = Optional.empty();
    this.conversionFactor = conversionFactor;
  }
  public SignalValue(BaseStatusSignal signal, BaseStatusSignal signalSlope, double conversionFactor) {
    this.signal = signal;
    this.signalSlope = Optional.of(signalSlope);
    this.conversionFactor = conversionFactor;
  }

  /**
   * Updates cached value based on the StatusSignals.
   * Cached value can also be retreived using .get().
   */
  public void updateCachedValue() {
    if (signalSlope.isPresent()) {
      cachedValue = BaseStatusSignal.getLatencyCompensatedValueAsDouble(
        signal, 
        signalSlope.get()
      );
    } else {
      cachedValue = signal.getValueAsDouble();
    }
  }

  public double get() {
    return cachedValue * conversionFactor;
  }

  public BaseStatusSignal getSignal() {
    return signal;
  }

  public Optional<BaseStatusSignal> getSignalSlope() {
    return signalSlope;
  }
}
