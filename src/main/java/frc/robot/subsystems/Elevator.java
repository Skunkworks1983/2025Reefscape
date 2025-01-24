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

  public Command getMoveToPositionCommand(double targetHeightMeters) {
    double[] startPosition = new double[0];
    double[] startVelocity = new double[0];
    Timer[] timeElapsed = new Timer[0];
    return Commands.startRun(
      () -> {
        timeElapsed[0] = new Timer();
        startPosition[0] = getElevatorPositionMeters();
        startVelocity[0] = getElevatorVelocityMeters();
      }, () -> {
        State motionProfileResult = motionProfile.calculate(
          timeElapsed[0].get(),
          new State(
            startPosition[0],
            startVelocity[0]
          ),
          new State(
            targetHeightMeters,
            0.0 // We want the elevator to stop moving
          )
        );

        motor.set(
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
    double[] currentTargetPosition = new double[0];
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
        motor.set(velocity);
      }
    );
  }
}
