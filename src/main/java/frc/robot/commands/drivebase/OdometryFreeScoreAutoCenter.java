// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
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
public class OdometryFreeScoreAutoCenter extends SequentialCommandGroup {
  public OdometryFreeScoreAutoCenter(Drivebase drivebase, Elevator elevator, Wrist wrist, Collector collector) {

    double[] time = new double[1];
    time[0] = 0.0;
    double[] waitSeconds = new double[1];
    waitSeconds[0] = 2.0;    addCommands(
      Commands.runEnd(
        () -> {},
        () -> {}
      ).beforeStarting(
        () -> {
          time[0] = Timer.getFPGATimestamp();
          waitSeconds[0] = SmartDashboard.getNumber("Auto wait seconds", 0.0);
        }
      ).until(
        () -> {
          return (time[0] + waitSeconds[0]) < Timer.getFPGATimestamp();
        }
      ),
      new TrapezoidProfileDriveStraight(drivebase, Units.feetToMeters(5), true),
      new MoveEndEffector(elevator, wrist, Constants.EndEffectorSetpoints.CORAL_L1, () -> 0.0),
      // This will align to the right
      collector.expelCoralCommand(true, elevator::getEndEffectorSetpoint)
    );
  }
}
