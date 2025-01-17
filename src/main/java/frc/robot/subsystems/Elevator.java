// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Elevator extends SubsystemBase {

  SparkMax motor = new SparkMax(
    Constants.Elevator.MOTOR_ID, 
    MotorType.kBrushed
  );

  PIDController positionController = new PIDController(
    Constants.Elevator.PIDs.ELEVATOR_kP,
    Constants.Elevator.PIDs.ELEVATOR_kI,
    Constants.Elevator.PIDs.ELEVATOR_kD,
    Constants.Elevator.PIDs.ELEVATOR_kF
  );
  
  public Elevator() {}

  @Override
  public void periodic() {
    updateMotor();
  }

  private void updateMotor() {
    motor.set(positionController.calculate(getElevatorPositionMeters()));
  }

  public double getElevatorPositionMeters() {
    return motor.getEncoder().getPosition() * Constants.Elevator.ROTATIONS_TO_METERS;
  }

  public Command moveToPosition(double heightMeters) {
    return Commands.runOnce(
      () -> positionController.setSetpoint(heightMeters)
    );
  }
}
