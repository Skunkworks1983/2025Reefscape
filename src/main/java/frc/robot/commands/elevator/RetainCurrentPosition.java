// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

// This command maintains the current position of the elevator.
// The position of the elevator is measured on start. This means
// that if a command (like MoveToPosition) ends and this command
// starts, this command will stop the elevator from falling back down.
public class RetainCurrentPosition extends Command {
  double currentTargetPosition;
  Elevator elevator;
  public RetainCurrentPosition(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
   }

  @Override
  public void initialize() {
    currentTargetPosition = elevator.getElevatorPosition();
  }

  @Override
  public void execute() {
    double velocity = elevator.getPositionController()
      .calculate(
        elevator.getElevatorPosition(),
        currentTargetPosition
      );
    elevator.setMotor(velocity);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
