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

import frc.robot.commands.elevator.RetainCurrentPosition;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


// All positions are stored in meters, all velocities are in meters/seconds, 
// all accelerations are stored in meters/second/second.
public class Elevator extends SubsystemBase {

  SparkMax motor = new SparkMax(
    Constants.Elevator.MOTOR_ID, 
    MotorType.kBrushless
  );

  final TrapezoidProfile motionProfile = new TrapezoidProfile(
    new Constraints(
      Profile.MAX_VELOCITY,
      Profile.MAX_ACCELERATION
    )
  );

  public TrapezoidProfile getMotionProfile() {
    return motionProfile;
  }

  private double targetPosition = getElevatorPosition();

  PIDController positionController = new PIDController(
    Constants.Elevator.PIDs.ELEVATOR_kP,
    Constants.Elevator.PIDs.ELEVATOR_kI,
    Constants.Elevator.PIDs.ELEVATOR_kD,
    Constants.Elevator.PIDs.ELEVATOR_kF
  );

  public PIDController getPositionController() {
    return positionController;
  }

  public Elevator() {
    setDefaultCommand(getRetainTargetPositionCommand());

    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);

    motor.configure(config, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {}

  // Reminder: all positions are measured in meters
  public double getElevatorPosition() {
    return motor.getEncoder().getPosition() * Constants.Elevator.ROTATIONS_TO_METERS;
  }

  // Reminder: all velocities are measured in meters/second
  private double getElevatorVelocity() {
    return motor.getEncoder().getVelocity() * Constants.Elevator.ROTATIONS_TO_METERS;
  }

  // Input is a percentage that ranges from -1 to 1.
  public void setMotor(double power) {
    motor.set(power);
  }

  private boolean isAtSetpoint(){
    return Math.abs(getElevatorPosition() - targetPosition) 
      < Constants.Elevator.TOLORENCE_METERS_FOR_SETPOINT;
  }

  public Command getRetainTargetPositionCommand() {
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

  // This command will move the elevator to the target position if it is close enough.
  // If the elevator has not reached the target position, it will retain its current 
  // position instead.
  public Command retainReasonablePosition() {
    return Commands.either(
      getRetainTargetPositionCommand(),
      new RetainCurrentPosition(this),
      () -> isAtSetpoint()
    );
  }
}
