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

  PIDController positionController = new PIDController(
    Constants.Elevator.PIDs.ELEVATOR_kP,
    Constants.Elevator.PIDs.ELEVATOR_kI,
    Constants.Elevator.PIDs.ELEVATOR_kD,
    Constants.Elevator.PIDs.ELEVATOR_kF
  );

  public Elevator() {}

  @Override
  public void periodic() {
    setDefaultCommand(getRetainCurrentPositionCommand());
  }

  public double getElevatorPositionMeters() {
    return motor.getEncoder().getPosition() * Constants.Elevator.ROTATIONS_TO_METERS;
  }

  public double getElevatorVelocityMeters() {
    return motor.getEncoder().getVelocity() * Constants.Elevator.ROTATIONS_TO_METERS;
  }

  // Input ranges from -1 to 1.
  private void setMotor(double power) {
    motor.set(power);
  }

  public Command getMoveToPositionCommand(double targetHeightMeters) {
    Timer timeElapsed = new Timer();
    State startState = new State();
    State targetState = new State(targetHeightMeters, 0.0);
    timeElapsed.stop();
    return Commands.startRun(
      () -> {
        timeElapsed.restart();
        startState.position = getElevatorPositionMeters();
        startState.velocity = getElevatorVelocityMeters();
      }, () -> {
        State motionProfileResult = motionProfile.calculate(
          timeElapsed.get(), // Time is the only variable that changes
          startState, 
          targetState
        );

        setMotor(
          positionController.calculate(
            getElevatorPositionMeters(),
            motionProfileResult.position
          )
        );
      }
    ).until(
      () -> (Constants.Elevator.TOLORENCE_METERS > 
        Math.abs(targetHeightMeters - getElevatorPositionMeters()))
    );
  }

  // TODO: Check if this is the wanted behavior
  public Command getRetainCurrentPositionCommand(){
    double[] currentTargetPosition = new double[1];
    return Commands.startRun(
      () -> {
        currentTargetPosition[0] = getElevatorPositionMeters();
      },
      () -> {
        double velocity = positionController
          .calculate(
            getElevatorPositionMeters(),
            currentTargetPosition[0]
          );
        setMotor(velocity);
      }
    );
  }
}
