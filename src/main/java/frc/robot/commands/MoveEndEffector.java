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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveEndEffector extends ParallelCommandGroup {
  /** Creates a new MoveEndEffector. */
  Elevator elevator;
  Wrist wrist;
  public MoveEndEffector(Elevator elevator, Wrist wrist, EndEffectorSetpointConstants setpoint) {
    addRequirements(elevator);
    addRequirements(wrist);
    this.elevator = elevator;
    this.wrist = wrist;
    this.addCommands(
      new MoveWristToSetpoint(wrist, setpoint.wristSetpoint),
      new MoveElevatorToSetpointCommand(elevator, setpoint.elevatorSetpoint)
    );
  }
}
