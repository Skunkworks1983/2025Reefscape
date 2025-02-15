// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.odometry;

/** Add your docs here. */
public class Phoenix6DrivebaseState {
  final public double gyroAngle;
  final public Phoenix6SwerveModuleState[] swerveState;

  public Phoenix6DrivebaseState(
    double gyroAngle, 
    Phoenix6SwerveModuleState[] swerveState){
    this.swerveState = swerveState;
    this.gyroAngle = gyroAngle;
    

  }
}
