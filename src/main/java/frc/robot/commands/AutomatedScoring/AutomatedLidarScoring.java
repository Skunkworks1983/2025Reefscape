// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutomatedScoring;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.EndEffectorSetpointConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.drivebase.TeleopFeatureUtils;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutomatedLidarScoring extends SequentialCommandGroup {
  /** Creates a new AutomatedLidarScoring. */
  public AutomatedLidarScoring(
      Drivebase drivebase,
      Collector collector,
      DoubleSupplier getXMetersPerSecond,
      DoubleSupplier getYMetersPerSecond,
      boolean goingRight,
      double alignSpeed,
      Supplier<EndEffectorSetpointConstants> endEffectorSetpoint,
      BooleanSupplier expelButton) {
    boolean[] shouldRun = new boolean[1];
    addCommands(
        drivebase.getSwerveAlignCoral(
            getXMetersPerSecond,
            getYMetersPerSecond,
            goingRight,
            alignSpeed).beforeStarting(() -> {
              Rotation2d targetHeading = TeleopFeatureUtils.getCoralCycleAngleNoOdometry(true,
                  drivebase.getCachedGyroHeading());
              boolean goingRightInRobotRelative = drivebase.goingRightInRobotRelative(goingRight, targetHeading);
              // Do NOT run alignment if the lidar we would use for alignment is currently showing a long distance (i.e. "is tripped").
              shouldRun[0] = goingRightInRobotRelative ? !drivebase.getDualLidar().isLidarRightTripped.getAsBoolean()
                  : !drivebase.getDualLidar().isLidarLeftTripped.getAsBoolean();
              if (!shouldRun[0]) {
                System.out.println("Canceling Auto Scoring Because Lidar is not triggered");
              }
            }).onlyIf(() -> {
              return shouldRun[0];
            }),
        Commands.waitUntil(
            () -> {
              EndEffectorSetpointConstants constants = endEffectorSetpoint.get();
              return ((constants == Constants.EndEffectorSetpoints.CORAL_L2)
                  || (constants == Constants.EndEffectorSetpoints.CORAL_L3)) && expelButton.getAsBoolean();
            }).onlyIf(() -> {
              return shouldRun[0];
            }),
        Commands.waitSeconds(0.1).onlyIf(() -> {
          return shouldRun[0];
        }),
        collector.expelCoralCommand(
            true,
            endEffectorSetpoint).withTimeout(2).onlyIf(() -> {
              return shouldRun[0];
            }));
  }
}
