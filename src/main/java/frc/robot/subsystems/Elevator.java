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
  
  Timer timeElapsed;
  double targetPosition = Constants.Elevator.Setpoints.FLOOR_POSITION_METERS;

  public Elevator() {}

  @Override
  public void periodic() {
    updateMotor();
  }

  private void updateMotor() {
    boolean atSetpoint = (Constants.Elevator.TOLORENCE_METERS >
      Math.abs(
        targetPosition - getElevatorPositionMeters()
      )
    );
    double currentTargetPosition;
    if(atSetpoint) {
      currentTargetPosition = targetPosition;
    } else {
       State motionProfileResult = motionProfile.calculate(
        timeElapsed.get(), 
        new State(
          getElevatorPositionMeters(),
          getElevatorVelocityMeters()
        ),
        new State(
          targetPosition,
          0.0 // We want the elevator to stop moving
        )
      );
      currentTargetPosition = motionProfileResult.position;
    }

    double velocity = positionController.calculate(
        getElevatorVelocityMeters(),
        currentTargetPosition 
    );
    motor.set(velocity);
  }

  public double getElevatorPositionMeters() {
    return motor.getEncoder().getPosition() * Constants.Elevator.ROTATIONS_TO_METERS;
  }

  public double getElevatorVelocityMeters() {
    return motor.getEncoder().getVelocity() * Constants.Elevator.ROTATIONS_TO_METERS;
  }

  public Command getMoveTopositionCommand(double heightMeters) {
    return Commands.runOnce(
      () -> {
        timeElapsed = new Timer();
        targetPosition = heightMeters;
      }
    );
  }
}
