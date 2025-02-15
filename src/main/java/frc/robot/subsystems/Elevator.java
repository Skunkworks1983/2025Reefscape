// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.utils.SmartPIDController;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


// All positions are stored in meters, all velocities are in meters/seconds, 
// all accelerations are stored in meters/second/second.
public class Elevator extends SubsystemBase {

  private TalonFX motor = new TalonFX(Constants.Elevator.MOTOR_ID);

  private double targetPosition = getElevatorPosition();

  SmartPIDController positionController = new SmartPIDController(
    Constants.Elevator.PIDs.ELEVATOR_kP,
    Constants.Elevator.PIDs.ELEVATOR_kI,
    Constants.Elevator.PIDs.ELEVATOR_kD,
    "Elevator",
    Constants.Elevator.PIDs.SMART_PID_ENABLED
  );

  public PIDController getPositionController() {
    return positionController;
  }

  public Elevator() {
    setDefaultCommand(retainTargetPositionCommand());

    TalonFXConfiguration config = new TalonFXConfiguration();

    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {}

  // Reminder: all positions are measured in meters
  public double getElevatorPosition() {
    return motor.getPosition().getValueAsDouble() * Constants.Elevator.ROTATIONS_TO_METERS;
  }

  // Reminder: all velocities are measured in meters/second
  public double getElevatorVelocity() {
    return motor.getVelocity().getValueAsDouble() * Constants.Elevator.ROTATIONS_TO_METERS;
  }

  // Input is a percentage that ranges from -1 to 1.
  public void setMotor(double speed) {
    motor.set(speed);
  }

  public boolean isAtSetpoint(){
    return Math.abs(getElevatorPosition() - targetPosition) 
      < Constants.Elevator.TOLORENCE_METERS_FOR_SETPOINT;
  }

  // This command does not use a motion profile. It is only for maintaing
  // positions and moving very small distances after 
  // MoveToPositionCommand has done almost all of the work.
  public Command retainTargetPositionCommand() {
    return run(
      () -> {
        double velocity = positionController
          .calculate(
            getElevatorPosition(),
            targetPosition
          );
        setMotor(velocity);
      }
    );
  }
}
