// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.odometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.Constants;

/** Add your docs here. */
public class Phoenix6SwerveModuleState {
  // All of these are in rotations
  public final double driveMotorPosition;
  public final double driveMotorVelocity;
  public final double turnMotorAngle;
  public Phoenix6SwerveModuleState(
    double driveMotorPosition,
    double driveMotorVelocity,
    double turnMotorAngle
  ) {
    this.driveMotorPosition = driveMotorPosition;
    this.driveMotorVelocity = driveMotorVelocity;
    this.turnMotorAngle = turnMotorAngle;
  }

  public SwerveModulePosition getSwerveModulePosition(){
    return new SwerveModulePosition(
      driveMotorPosition / Constants.Drivebase.Info.REVS_PER_METER,
      Rotation2d.fromRotations(turnMotorAngle)
    );
  }

  public SwerveModuleState getSwerveModuleState(){
    return new SwerveModuleState(
      driveMotorVelocity / Constants.Drivebase.Info.REVS_PER_METER,
      Rotation2d.fromRotations(turnMotorAngle)
    );
  }

}
