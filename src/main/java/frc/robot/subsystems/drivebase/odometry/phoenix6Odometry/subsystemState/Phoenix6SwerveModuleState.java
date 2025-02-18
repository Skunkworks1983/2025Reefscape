// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemSignals.SubsystemSignal;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemSignals.Phoenix6SwerveModuleSignal.Field;

public class Phoenix6SwerveModuleState extends SubsystemState<Field> {

  public Phoenix6SwerveModuleState(SubsystemSignal<Field> signalSubsystem) {
    super(signalSubsystem);
  }

  public SwerveModulePosition getSwerveModulePosition(){
    return new SwerveModulePosition(
      super.getValue(Field.DRIVE_POSITION) / Constants.Drivebase.Info.REVS_PER_METER,
      Rotation2d.fromRotations(super.getValue(Field.TURN_POSITION))
    );
  }

  public SwerveModuleState getSwerveModuleState(){
    return new SwerveModuleState(
      super.getValue(Field.DRIVE_VELOCITY) / Constants.Drivebase.Info.REVS_PER_METER,
      Rotation2d.fromRotations(super.getValue(Field.TURN_POSITION))
    );
  }

}
