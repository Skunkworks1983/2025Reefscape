// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Elevator.Profile;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;


// All positions are stored in meters, all velocities are in meters/seconds, 
// all accelerations are stored in meters/second/second.
public class Elevator extends SubsystemBase {

  SparkMax motor = new SparkMax(
    Constants.Elevator.MOTOR_ID, 
    MotorType.kBrushed
  );

  final TrapezoidProfile motionProfile = new TrapezoidProfile(
    new Constraints(
      Profile.MAX_VELOCITY,
      Profile.MAX_ACCELERATION
    )
  );

  double targetPosition = getElevatorPosition();

  PIDController positionController = new PIDController(
    Constants.Elevator.PIDs.ELEVATOR_kP,
    Constants.Elevator.PIDs.ELEVATOR_kI,
    Constants.Elevator.PIDs.ELEVATOR_kD,
    Constants.Elevator.PIDs.ELEVATOR_kF
  );

  public Elevator() {
    setDefaultCommand(getRetainTargetPositionCommand());
  }

  @Override
  public void periodic() {}

  // Reminder: all positions are measured in meters
  private double getElevatorPosition() {
    return motor.getEncoder().getPosition() * Constants.Elevator.ROTATIONS_TO_METERS;
  }

  // Reminder: all velocities are measured in meters/second
  private double getElevatorVelocity() {
    return motor.getEncoder().getVelocity() * Constants.Elevator.ROTATIONS_TO_METERS;
  }

  // Input is a percentage that ranges from -1 to 1.
  private void setMotor(double power) {
    motor.set(power);
  }

  private boolean isAtSetpoint(){
    return Math.abs(getElevatorPosition() - targetPosition) 
      < Constants.Elevator.TOLORENCE_METERS_FOR_SETPOINT;
  }

  // This command moves to the elevator from its current position to the argument
  // targetHeight (meters). This command will end when the elevator is within some
  // tolorence of the desired position.
  public Command getMoveToPositionCommand(double targetHeight) {
    Timer timeElapsed = new Timer();
    State startState = new State(); // This will be set in the start of the command
    State targetState = new State(targetHeight, 0.0);
    timeElapsed.stop();
    return Commands.startRun(
      () -> {
        timeElapsed.restart();
        targetPosition = targetHeight;
        startState.position = getElevatorPosition();
        startState.velocity = getElevatorVelocity();
      }, () -> {
        State motionProfileResult = motionProfile.calculate(
          timeElapsed.get(), // Time is the only variable that changes throughout run
          startState, 
          targetState
        );

        setMotor(
          positionController.calculate(
            getElevatorPosition(),
            motionProfileResult.position
          )
        );
      }
    ).until(
      () -> (Constants.Elevator.TOLERENCE_METERS_FOR_MOVE_TO_POSITION > 
        Math.abs(targetHeight - getElevatorPosition()))
    );
  }

  // This command maintains the current position of the elevator.
  // The position of the elevator is measured on start. This means
  // that if a command (like MoveToPosition) ends and this command
  // starts, this command will stop the elevator from falling back down.
  public Command getRetainCurrentPositionCommand() {
    double[] currentTargetPosition = new double[1];
    return Commands.startRun(
      () -> {
        currentTargetPosition[0] = getElevatorPosition();
      },
      () -> {
        double velocity = positionController
          .calculate(
            getElevatorPosition(),
            currentTargetPosition[0]
          );
        setMotor(velocity);
      }
    );
  }

  public Command getRetainTargetPositionCommand() {
    return Commands.run(
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

  // This command will move the elevator to the target position if it is close enough.
  // If the elevator has not reached the target position, it will retain its current 
  // position instead.
  public Command retainReseaonablePosition() {
    return Commands.either(
      getRetainTargetPositionCommand(),
      getRetainCurrentPositionCommand(),
      () -> isAtSetpoint()
    );
  }
}
