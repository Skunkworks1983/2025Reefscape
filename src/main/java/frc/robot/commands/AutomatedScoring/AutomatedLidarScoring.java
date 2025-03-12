// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutomatedScoring;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.constants.EndEffectorSetpointConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.drivebase.Drivebase;

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
      double alignSpeed,
      boolean goingRight,
      double backSeconds,
      String name,
      Supplier<EndEffectorSetpointConstants> endEffectorSetpoint) {
    addCommands(
      drivebase.getSwerveAlignCoral(
        getXMetersPerSecond,
        getYMetersPerSecond,
        alignSpeed,
        goingRight,
        backSeconds,
        name
      ),
      Commands.waitUntil(
        () -> {
          EndEffectorSetpointConstants constants = endEffectorSetpoint.get();
          if(constants == Constants.EndEffectorSetpoints.CORAL_L2) {
            return true;
          }
          if(constants == Constants.EndEffectorSetpoints.CORAL_L3) {
            return true;
          }
          return false;
        }
      ),
      collector.expelCoralCommand(
        true, 
        endEffectorSetpoint
      )
    );
  }
}
