// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.error;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class ErrorCommandGenerator {
  public static Command getErrorCommand(
    ErrorGroup errorGroupHandler,
    DiagnosticSubsystem[] errorSubsystem
  ) {
    Command[] errorCommandArray = new Command[errorSubsystem.length];
    for(int i = 0; i < errorSubsystem.length; i++) {
      errorCommandArray[i] = errorSubsystem[i].getErrorCommand(errorGroupHandler);
    }
    return Commands.sequence(errorCommandArray);
  }
}
