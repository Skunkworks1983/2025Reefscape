// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.odometry.subsystemState;

import frc.robot.utils.odometry.subsystemSignals.SubsystemSignal;

/** Add your docs here. */
public abstract class SubsystemState<FIELD> {

  SubsystemSignal<FIELD> signalSubsystem;
  public SubsystemState(SubsystemSignal<FIELD> signalSubsystem) {
    this.signalSubsystem = signalSubsystem;
  }

  public double getValue(FIELD field) {
    return signalSubsystem.signalValueMap.get(field).get();
  }
}
