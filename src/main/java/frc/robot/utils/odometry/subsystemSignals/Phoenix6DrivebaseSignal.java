// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.odometry.subsystemSignals;

import com.ctre.phoenix6.StatusSignal;

import frc.robot.constants.Constants;

/** Add your docs here. */
public class Phoenix6DrivebaseSignal {
  private Phoenix6DrivebaseSignal[] swerveModules = 
    new Phoenix6DrivebaseSignal[Constants.Drivebase.MODULES.length];
    private StatusSignal<Angle> gyroStatusSignal;
}
