// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivebase extends SubsystemBase {

  SwerveModule swerveModules[] = new SwerveModule[]{
    new SwerveModule(),
    new SwerveModule(),
    new SwerveModule(),
    new SwerveModule()
  };

  public Drivebase() {}

  @Override
  public void periodic() {
  }

  public Command getSwerveTeleopCommand(
    DoubleSupplier xMetersPerSecond,
    DoubleSupplier yMetersPerSecond, 
    Supplier<Rotation2d> rotationPerSecond
  ) {
    return Commands.runEnd(
      () -> {},
      () -> {}


    );
  }
}
