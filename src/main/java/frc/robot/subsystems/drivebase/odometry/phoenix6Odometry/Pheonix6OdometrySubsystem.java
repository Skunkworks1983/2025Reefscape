// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase.odometry.phoenix6Odometry;

import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState.PhoenixSubsystemState;

/** 
 * This interface is used to provide a consistent way to get the state of a subsystem
 * that uses phoenix 6 odometry because all the state of the subsystem is stored in a
 * PhoenixSubsystemState object. 
 */
public interface Pheonix6OdometrySubsystem<FIELD> {
  public PhoenixSubsystemState<FIELD> getState();
}
