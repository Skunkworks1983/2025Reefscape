// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.MoveElevatorToSetpointCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.constants.Constants;
import frc.robot.constants.EndEffectorSetpointConstants;

// Moves wrist and elevator
public class MoveEndEffector extends SequentialCommandGroup {
  boolean wristUp;
  boolean elevatorUp;
  public MoveEndEffector(
    Elevator elevator,
    Wrist wrist,
    EndEffectorSetpointConstants setpoint,
    DoubleSupplier getOffset
  ) {
    this(
      elevator,
      wrist,
      setpoint,
      getOffset,
      Constants.Elevator.Profile.MAX_VELOCITY,
      Constants.Elevator.Profile.MAX_ACCELERATION
    );
  }
  public MoveEndEffector(
    Elevator elevator,
    Wrist wrist,
    EndEffectorSetpointConstants setpoint,
    DoubleSupplier getOffset,
    double elevatorVelocityOveride,
    double elevatorAcceletationOveride
  ) {
    wristUp = false;
    elevatorUp = false;
    addCommands(
      new MoveWristToSetpoint(wrist, setpoint.stowSetpoint).finallyDo(interrupted -> {
        wristUp = !interrupted;
      }),
      new MoveElevatorToSetpointCommand(
        elevator,
        () -> setpoint.elevatorSetpoint + getOffset.getAsDouble(),
        elevatorVelocityOveride,
        elevatorAcceletationOveride
      ).beforeStarting(() -> {
        if (!wristUp) {
          this.cancel();
        }
      }).finallyDo(interrupted -> {
        elevatorUp = !interrupted;
      }),
      new MoveWristToSetpoint(wrist, setpoint.wristSetpoint).beforeStarting(() -> {
        if (!(elevatorUp && wristUp)) {
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
