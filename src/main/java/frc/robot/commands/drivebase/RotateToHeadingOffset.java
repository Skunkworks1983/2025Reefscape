// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivebase.Drivebase;

public class RotateToHeadingOffset extends Command {

  Drivebase drivebase;
  double headingOffset;
  PIDController rotController;

  public RotateToHeadingOffset(Drivebase drivebase, Rotation2d headingOffset) {
    this.drivebase = drivebase;
    this.headingOffset = headingOffset.getDegrees();
    rotController = new PIDController(
        Constants.Drivebase.PIDs.HEADING_CONTROL_kP,
        Constants.Drivebase.PIDs.HEADING_CONTROL_kI,
        Constants.Drivebase.PIDs.HEADING_CONTROL_kD
      );
    rotController.enableContinuousInput(0, 360); // The PID controller is in degrees\
    rotController.setTolerance(1.0);
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    rotController.setSetpoint(drivebase.getCachedGyroHeading().getDegrees() + this.headingOffset);
  }

  @Override
  public void execute() {
    drivebase.drive(0.0, 0.0, rotController.calculate(drivebase.getCachedGyroHeading().getDegrees()), true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return rotController.atSetpoint();
  }
}
