// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.MoveElevatorToSetpointCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.constants.EndEffectorSetpointConstants;

// Moves wrist and elevator
public class MoveEndEffector extends SequentialCommandGroup {
  boolean wristUp;
  boolean elevatorUp;
  public MoveEndEffector(
    Elevator elevator,
    Wrist wrist,
    EndEffectorSetpointConstants setpoint
  ) {
    wristUp = false;
    elevatorUp = false;
    addCommands(
      new MoveWristToSetpoint(wrist, setpoint.stowSetpoint).beforeStarting(() -> {
        System.out.println("running wrist");
      }).finallyDo(interrupted -> {
        wristUp = !interrupted;
        System.out.println("done running wrist, interupted " + interrupted);
      }),
      new MoveElevatorToSetpointCommand(elevator, setpoint.elevatorSetpoint).beforeStarting(() -> {
        if (!wristUp) {
          System.out.println("cancelling first elevator command");
          this.cancel();
        }
      }).finallyDo(interrupted -> {
        elevatorUp = !interrupted;
      }),
      new MoveWristToSetpoint(wrist, setpoint.wristSetpoint).beforeStarting(() -> {
        if (!(elevatorUp && wristUp)) {
          System.out.println("cancelling second wrist command");
          this.cancel();
        }
      }).finallyDo(
        interrupted -> {
          if (!interrupted) {
            elevator.setEndEffectorSetpoint(setpoint);
            System.out.println("Move end Effector finished: uninterupted");
          }
          else{
            System.out.println("Move end Effector finished: interupted");
          }
        }
      )
    );
  }
}
