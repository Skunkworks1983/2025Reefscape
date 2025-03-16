// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.collector.ExpelWhenAtElevatorPos;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveEndEffectorAndScoreNet extends ParallelCommandGroup {
  /** Creates a new MoveEndEffectorAndScoreNet. */
  public MoveEndEffectorAndScoreNet(
    Collector collector,
    Elevator elevator,
    Wrist wrist
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveEndEffector(
        elevator,
        wrist,
        Constants.EndEffectorSetpoints.ALGAE_NET,
        () -> 0,
        Constants.Elevator.Profile.MAX_VELOCITY_NET,
        Constants.Elevator.Profile.MAX_ACCELERATION_NET
      ),
      new ExpelWhenAtElevatorPos(collector, elevator::getElevatorPosition, 19.0)
    );
  }
}
