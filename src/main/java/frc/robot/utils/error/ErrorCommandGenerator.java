// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.error;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// this is used by robot to create a singular command to run to test all errors. 
// It takes in a errorGroup to pass into each subsystems command and a list of subsystems to get commands from

public class ErrorCommandGenerator {
  public static Command getErrorCommand(
    ErrorGroup errorGroup,
    List<Optional<? extends DiagnosticSubsystem>> errorSubsystems
  ) {
    
    ArrayList<Command> errorCommandArray = new ArrayList<>();
    
    for(int i = 0; i < errorSubsystems.size(); i++) {
      if(errorSubsystems.get(i).isPresent()) {
        errorCommandArray.add(errorSubsystems.get(i).get().getErrorCommand(errorGroup));
      }
    }

    return Commands.sequence(errorCommandArray.toArray(new Command[0]));
  }
}
