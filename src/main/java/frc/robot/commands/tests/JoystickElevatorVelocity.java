// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class JoystickElevatorVelocity extends Command {
  DoubleSupplier elevatorPositionChange;

  Elevator elevator;

  public JoystickElevatorVelocity (
    Elevator elevator,
    DoubleSupplier elevatorPositionChange
  ) {
    addRequirements(elevator);
    this.elevatorPositionChange = elevatorPositionChange;
    this.elevator = elevator;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double elevatorVelocity = elevatorPositionChange.getAsDouble() 
      * Constants.Testing.ELEVATOR_MAX_SPEED;

    elevator.setSpeeds(elevatorVelocity);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
