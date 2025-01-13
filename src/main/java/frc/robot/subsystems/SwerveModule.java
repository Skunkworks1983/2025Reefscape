// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class SwerveModule extends SubsystemBase {

  int driveModuleId;
  int turnModuleId;
  public SwerveModule(Constants.Drivebase.SwerveModuleConstants swerveConstants) {
    this(swerveConstants.driveMotorId, swerveConstants.turnMotorId);
  }
  
  public SwerveModule(int driveModuleId, int turnModuleId) {
    this.driveModuleId = driveModuleId;
    this.turnModuleId = turnModuleId;
  }

  @Override
  public void periodic() {}

  public void setModuleSetpoints() {
    setModuleDriveSetpoint();
    setModuleTurnSetpoint();
  }

  // TODO: add code
  public void setModuleTurnSetpoint() {}


  // TODO: add code
  public void setModuleDriveSetpoint() {}
}
