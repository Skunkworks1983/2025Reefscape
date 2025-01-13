// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Drivebase extends SubsystemBase {

  SwerveModule swerveModules[] = new SwerveModule[4];

  public Drivebase() {
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i] = new SwerveModule(Constants.Drivebase.MODULES[i]);
    }

  }

  @Override
  public void periodic() {}

  // TODO: add docstring
  private void setDrive(double xMetersPerSecond,
  double ymetersPerSecond, Rotation2d rotationPerSecond) {

  }

  public Command getSwerveTeleopCommand(
    DoubleSupplier xMetersPerSecond,
    DoubleSupplier yMetersPerSecond, 
    Supplier<Rotation2d> rotationPerSecond
  ) {
    return Commands.runEnd(
      () -> {
        setDrive(
          xMetersPerSecond.getAsDouble(),
          yMetersPerSecond.getAsDouble(),
          rotationPerSecond.get()
        );
      },
      () -> {
        setDrive(
          0,
          0,
          Rotation2d.kZero // check required rotation
        );
      }
    );
  }
}
