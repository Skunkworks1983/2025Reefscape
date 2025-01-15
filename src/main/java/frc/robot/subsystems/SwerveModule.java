// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase {

  TalonFX driveModule;
  TalonFX turnModule;

  public SwerveModule(SwerveModuleConstants swerveConstants) {
    this(swerveConstants.driveMotorId, swerveConstants.turnMotorId);
  }
  
  public SwerveModule(int driveModuleId, int turnModuleId) {
    this.driveModule = new TalonFX(driveModuleId);
    this.turnModule = new TalonFX(turnModuleId);
  }

  @Override
  public void periodic() {}

  // TODO: add arguments to these functions
  public void setModuleSetpoints() {
    setModuleDriveSetpoint();
    setModuleTurnSetpoint();
  }

  // TODO: add code
  public void setModuleTurnSetpoint() {}


  // TODO: add code
  public void setModuleDriveSetpoint() {}
}