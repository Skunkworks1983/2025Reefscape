// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.elevator.MoveElevatorToSetpointCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.constants.EndEffectorSetpointConstants;

// Moves wrist and elevator
public class MoveEndEffector extends ParallelCommandGroup {
  public MoveEndEffector(
    Elevator elevator,
    Wrist wrist,
    EndEffectorSetpointConstants setpoint
  ) {
    addCommands(
      new MoveWristToSetpoint(wrist, setpoint.wristSetpoint),
      new MoveElevatorToSetpointCommand(elevator, setpoint.elevatorSetpoint)
    );
  }
}
