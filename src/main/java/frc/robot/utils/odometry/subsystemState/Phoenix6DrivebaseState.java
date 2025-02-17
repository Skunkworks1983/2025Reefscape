// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.odometry.subsystemState;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.odometry.subsystemSignals.Phoenix6DrivebaseSignal;
import frc.robot.utils.odometry.subsystemSignals.Phoenix6DrivebaseSignal.Field;

public class Phoenix6DrivebaseState extends SubsystemState<Field> {

  public Phoenix6DrivebaseState(Phoenix6DrivebaseSignal subsystem) {
    super(subsystem);
  }

  public Rotation2d getGyroAngle() {
    return Rotation2d.fromRotations(super.getValue(Field.GYRO));
  }
}
