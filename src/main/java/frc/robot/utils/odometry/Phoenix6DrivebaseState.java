// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.odometry;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Phoenix6DrivebaseState {
  final public Rotation2d gyroAngle;
  final public Phoenix6SwerveModuleState[] swerveState;

  public Phoenix6DrivebaseState(
    Rotation2d gyroAngle, 
    Phoenix6SwerveModuleState[] swerveState){
    this.swerveState = swerveState;
    this.gyroAngle = gyroAngle;
  }
}
