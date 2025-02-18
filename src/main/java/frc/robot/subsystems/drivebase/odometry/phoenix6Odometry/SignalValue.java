package frc.robot.subsystems.drivebase.odometry.phoenix6Odometry;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Signal value represents a signal (essensially an advanced supplier for sensors), an optional 
// derivative signal (to estimate where the most recent value is based on the change in time from the
// most recent sample), and the most recent sampled value of the previous two values.
public class SignalValue {
  final BaseStatusSignal signal;
  final Optional<BaseStatusSignal> signalSlope;
  double cachedValue;

  public SignalValue(BaseStatusSignal signal) {
    this.signal = signal;
    this.signalSlope = Optional.empty();
  }
  public SignalValue( BaseStatusSignal signal, BaseStatusSignal signalSlope) {
    this.signal = signal;
    this.signalSlope = Optional.of(signalSlope);
  }

  // Updates cached value based on the StatusSignals. Also returns the cached value.
  // Cached value can also be retreived using .get().
  public double updateCachedValue() {
    if(signalSlope.isPresent()){
      cachedValue = signal.getValueAsDouble();
    } else {
      cachedValue = BaseStatusSignal.getLatencyCompensatedValueAsDouble(
        signal, 
        signalSlope.get()
      );
    }
    return cachedValue;
  }

  public double get() {
    return cachedValue;
  }

  public BaseStatusSignal getSignal() {
    return signal;
  }

  public Optional<BaseStatusSignal> getSignalSlope() {
    return signalSlope;
  }
}
