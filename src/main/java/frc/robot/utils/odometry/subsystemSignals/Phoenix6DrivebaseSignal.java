// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.odometry.subsystemSignals;

import com.ctre.phoenix6.BaseStatusSignal;

import frc.robot.constants.Constants;
import frc.robot.utils.odometry.subsystemState.Phoenix6DrivebaseState;
  
public class Phoenix6DrivebaseSignal extends SubsystemSignal
  <frc.robot.utils.odometry.subsystemSignals.Phoenix6DrivebaseSignal.Field> {

  public Phoenix6SwerveModuleSignal[] modules 
    = new Phoenix6SwerveModuleSignal[Constants.Drivebase.MODULES.length];

  public Phoenix6DrivebaseSignal(BaseStatusSignal gyroSignal) {

  }

  @Override public void updateCachedValues() {
    super.updateCachedValues();
    for(Phoenix6SwerveModuleSignal module : modules){
      module.updateCachedValues();
    }
  };
  public static enum Field {
    GYRO
  }

  @Override
  public Phoenix6DrivebaseState getState() {
    return new Phoenix6DrivebaseState(this);
  }
}