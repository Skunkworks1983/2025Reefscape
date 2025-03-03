// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class joystickEndEffector extends Command {
  double wristPosition = 0.0;
  double elevatorPosition = 0.0;
  DoubleSupplier wristPositionChange;
  DoubleSupplier elevatorPositionChange;

  PositionVoltage positionVoltageWrist = new PositionVoltage(0);
  PositionVoltage positionVoltageElevator = new PositionVoltage(0);

  Wrist wrist;
  Elevator elevator;
  double time;

  public joystickEndEffector(
    Wrist wrist,
    Elevator elevator,
    DoubleSupplier wristPositionChange,
    DoubleSupplier elevatorPositionChange
  ) {
    addRequirements(wrist, elevator);
    this.wristPositionChange = wristPositionChange;
    this.elevatorPositionChange = elevatorPositionChange;
    this.wrist = wrist;
    this.elevator = elevator;
  }

  @Override
  public void initialize() {
    time = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    double deltaTime = Timer.getFPGATimestamp() - time;
    time = Timer.getFPGATimestamp();
    wristPosition += wristPositionChange.getAsDouble();
    elevatorPosition += elevatorPositionChange.getAsDouble();

    positionVoltageElevator.Position = elevatorPosition;
    positionVoltageWrist.Position = wristPosition;

    elevator.setElevatorMotorControl(positionVoltageElevator);
    wrist.setWristMotorControl(positionVoltageWrist);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
