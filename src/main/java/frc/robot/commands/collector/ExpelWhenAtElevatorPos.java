// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collector;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Collector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExpelWhenAtElevatorPos extends SequentialCommandGroup {
  /** This command is used to expel algae based on if the elevator has passed a certan position. This
   * is Currently being used to add velocity onto a launched peice of algae by releasing it halfway
   * on the way up to L4 position to make sure the algae makes it into the reef.
  */

  public ExpelWhenAtElevatorPos(
    Collector collector,
    DoubleSupplier elevatorPos,
    Double ejectPos
  ) {
    addCommands(
      Commands.waitUntil(
        () -> {
          return elevatorPos.getAsDouble() > ejectPos;
        }
      ),
      collector.expelAlgaeCommand(true).withTimeout(2)
    );
  }
}
