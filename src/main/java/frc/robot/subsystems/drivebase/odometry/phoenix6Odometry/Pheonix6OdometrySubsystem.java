// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase.odometry.phoenix6Odometry;

import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState.SubsystemState;

/** Add your docs here. */
public interface Pheonix6OdometrySubsystem<FIELD> {
  public SubsystemState<FIELD> getState();
}
