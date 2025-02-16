// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  private DigitalInput bottomLimitSwitch = new DigitalInput(Constants.Elevator.BOTTOM_LIMIT_SWITCH_ID);
  private DigitalInput topLimitSwitch = new DigitalInput(Constants.Elevator.TOP_LIMIT_SWITCH_ID);

  private double targetPosition;
  private double lastSpeed;

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
    TalonFXConfiguration config = new TalonFXConfiguration();
    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);

    targetPosition = getElevatorPosition();
    setDefaultCommand(retainTargetPositionCommand());
  }

  @Override
  public void periodic() {
    if(getBottomLimitSwitch()) {
      motor.setPosition(0.0);
      if(lastSpeed < 0.0) {
        setMotorSafe(0.0);
      }
    } else if(getTopLimitSwitch()) {
      motor.setPosition(Constants.Elevator.MAX_HEIGHT_CARRIAGE * Constants.Elevator.METERS_TO_MOTOR_ROTATIONS);
      if(lastSpeed > 0.0) {
        setMotorSafe(0.0);
      }
    }
    System.out.println(getElevatorPosition());
  }

  // Reminder: all positions are measured in meters
  public double getElevatorPosition() {
    return motor.getPosition().getValueAsDouble() * Constants.Elevator.MOTOR_ROTATIONS_TO_METERS;
  }

  // Reminder: all velocities are measured in meters/second
  public double getElevatorVelocity() {
    return motor.getVelocity().getValueAsDouble() * Constants.Elevator.MOTOR_ROTATIONS_TO_METERS;
  }

  // Inverted because limit switches return true until tripped
  public boolean getBottomLimitSwitch() {
    return !bottomLimitSwitch.get();
  }

  // Inverted because limit switches return true until tripped
  public boolean getTopLimitSwitch() {
    return !topLimitSwitch.get();
  }

  // Input is a percentage that ranges from -1 to 1.
  public void setMotorSafe(double speed) {
    if((getBottomLimitSwitch() && speed < 0.0) || (getTopLimitSwitch() && speed > 0.0)) {
      lastSpeed = 0.0;
    }
    else {
      lastSpeed = speed;
    }
    motor.set(lastSpeed);
  }

  public boolean isAtSetpoint() {
    return Math.abs(getElevatorPosition() - targetPosition) 
      < Constants.Elevator.TOLORENCE_METERS_FOR_SETPOINT;
  }

  public void setTargetPosition(double newTargetPosition) {
    targetPosition = newTargetPosition;
  }

  // This command does not use a motion profile. It is only for maintaing
  // positions and moving very small distances after 
  // MoveToPositionCommand has done almost all of the work.
  public Command retainTargetPositionCommand() {
    return Commands.startRun(
      () -> {
        setTargetPosition(getElevatorPosition());
      },
      () -> {
        double velocity = positionController
          .calculate(
            getElevatorPosition(),
            targetPosition
          );
        setMotorSafe(velocity);
      }
    );
  }
}
