// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.CollectorBloopSpeedCommand;
import frc.robot.constants.EndEffectorSetpointConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L4BloopCommand extends SequentialCommandGroup {
  /** Creates a new L4BloopCommand. */
  public L4BloopCommand(Elevator elevator, Wrist wrist, Collector collector, EndEffectorSetpointConstants position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.parallel(
        new MoveEndEffector(elevator, wrist, position),
        collector.intakeCoralCommand(true)
      ),
      new CollectorBloopSpeedCommand(collector, 0.0)
    );
  }
}
