// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

/** Wait for momentum to dissapate */
public class SwerveWaitCommand extends Command {

  private Timer timer = new Timer();
  private Rotation2d lastRecordedHeading;
  private boolean useHeadingControl = false;
  private Drivebase drivebase;
  private Supplier<Rotation2d> getHeading;
  private DoubleSupplier getYMetersPerSecond;
  private DoubleSupplier getXMetersPerSecond;

  public SwerveWaitCommand(
      Drivebase drivebase, 
      Supplier<Rotation2d> getHeading, 
      DoubleSupplier getXMetersPerSecond,
      DoubleSupplier getYMetersPerSecond) {
    
    this.getHeading = getHeading;
    this.getXMetersPerSecond = getXMetersPerSecond;
    this.getYMetersPerSecond = getYMetersPerSecond;

    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    if (timer.hasElapsed(Constants.Drivebase.SECONDS_UNTIL_HEADING_CONTROL) && !useHeadingControl) {
      lastRecordedHeading = getHeading.get();
      useHeadingControl = true;
    }

    if (useHeadingControl) {
      drivebase.drive(
        getXMetersPerSecond.getAsDouble(), 
        getYMetersPerSecond.getAsDouble(), 
        0, 
        true);
    } else {
      drivebase.drive(0, 0, 0, true);
    }

    // add else
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
