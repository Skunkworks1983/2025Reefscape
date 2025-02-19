// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemSignals;

import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.SignalValue;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState.Phoenix6SwerveModuleState;

public class Phoenix6SwerveModuleSignal extends SubsystemSignal
  <frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemSignals.Phoenix6SwerveModuleSignal.SwerveField> {

  public Phoenix6SwerveModuleSignal(
    SignalValue turnPosition,
    SignalValue turnVelocity,
    SignalValue drivePosition,
    SignalValue driveVelocity
  ) {
    super.signalValueMap.put(SwerveField.DRIVE_POSITION, drivePosition);
    super.signalValueMap.put(SwerveField.DRIVE_VELOCITY, driveVelocity);
    super.signalValueMap.put(SwerveField.TURN_POSITION, turnPosition);
    super.signalValueMap.put(SwerveField.TURN_VELOCITY, turnVelocity);
  }

  // The datatype that should be used for hashing SignalValues
  public static enum SwerveField {
    DRIVE_POSITION,
    DRIVE_VELOCITY, 
    TURN_POSITION,
    TURN_VELOCITY, 
  }

  @Override
  public Phoenix6SwerveModuleState getState() {
    return new Phoenix6SwerveModuleState(this);
  }
}
