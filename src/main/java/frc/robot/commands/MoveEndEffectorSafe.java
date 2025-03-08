// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.MoveElevatorToSetpointCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.constants.Constants;
import frc.robot.constants.EndEffectorSetpointConstants;

// Moves wrist and elevator
public class MoveEndEffectorSafe extends SequentialCommandGroup {
  boolean wristUp;
  boolean elevatorUp1;
  boolean elevatorUp2;
  public MoveEndEffectorSafe(
    Elevator elevator,
    Wrist wrist,
    EndEffectorSetpointConstants setpoint
  ) {
    wristUp = false;
    elevatorUp1 = false;
    elevatorUp2 = false;
    addCommands(
      new MoveWristToSetpoint(wrist, setpoint.stowSetpoint).finallyDo(interrupted -> {
        wristUp = !interrupted;
      }),
      new MoveElevatorToSetpointCommand(elevator, Math.min(elevator.getElevatorPosition() + 1.5, Constants.EndEffectorSetpoints.CORAL_L4.elevatorSetpoint)).beforeStarting(() -> {
        if(!wristUp) {
          this.cancel();
        }
      }).finallyDo(interrupted -> {
        elevatorUp1 = !interrupted;
      }),
      new MoveElevatorToSetpointCommand(elevator, setpoint.elevatorSetpoint).beforeStarting(() -> {
        if(!(elevatorUp1 && wristUp)) {
          this.cancel();
        }
      }).finallyDo(interrupted -> {
        elevatorUp2 = !interrupted;
      }),
      new MoveWristToSetpoint(wrist, setpoint.wristSetpoint).beforeStarting(() -> {
        if(!(elevatorUp2 && elevatorUp1 && wristUp)) {
          this.cancel();
        }
      })
    );
  }
}
