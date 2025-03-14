// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveEndEffector;
import frc.robot.commands.AutomatedScoring.AutomatedLidarScoring;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.drivebase.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OdometryFreeScoreAuto extends SequentialCommandGroup {
  public OdometryFreeScoreAuto(Drivebase drivebase, Elevator elevator, Wrist wrist, Collector collector, boolean isLeftSideOfBarge) {
    
    addCommands(
      new TrapezoidProfileDriveStraight(drivebase, Units.feetToMeters(4.0), true),
      new RotateToHeading(drivebase, Rotation2d.fromDegrees(isLeftSideOfBarge ? -60.0 : 60.0)),
      new TrapezoidProfileDriveStraight(drivebase, Units.feetToMeters(4.0), false),
      new MoveEndEffector(elevator, wrist, Constants.EndEffectorSetpoints.CORAL_L2),
      // This will align to the right
      new AutomatedLidarScoring(
        drivebase, 
        collector, 
        () -> 0.0, 
        () -> 0.0, 
        true, 
        0.3, 
        () -> Constants.EndEffectorSetpoints.CORAL_L2
      ),
      new MoveEndEffector(elevator, wrist, Constants.EndEffectorSetpoints.CORAL_STOW)
    );
  }
}
