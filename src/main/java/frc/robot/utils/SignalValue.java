package frc.robot.utils;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;

public class SignalValue
    <U extends Unit, U_PER_SEC extends PerUnit<U, TimeUnit>, MEAS extends Measure<U>, MEAS_PER_SEC extends Measure<U_PER_SEC>>
 {
  private StatusSignal<MEAS> m_statusSignal;
  private StatusSignal<MEAS_PER_SEC> m_statusSignalRate;
  private double m_value;

  public SignalValue(
    StatusSignal<MEAS> statusSignal,
    StatusSignal<MEAS_PER_SEC> statusSignalRate, 
    double value
  ) {
    m_statusSignal = statusSignal;
    m_statusSignalRate = statusSignalRate;
    m_value = value;
  }

  public StatusSignal<MEAS> getStatusSignal() {
    return m_statusSignal;
  }

  public StatusSignal<MEAS_PER_SEC> getstatusSignalRate() {
    return m_statusSignalRate;
  }

  public double getValue() {
    return m_value;
  }

  public void setStatusSignal(StatusSignal<MEAS> statusSignal) {
    m_statusSignal = statusSignal;
  }

  public void setstatusSignalRate(StatusSignal<MEAS_PER_SEC> second) {
    m_statusSignalRate = second;
  }

  public void setValue(double value) {
    m_value = value;
  }
}
