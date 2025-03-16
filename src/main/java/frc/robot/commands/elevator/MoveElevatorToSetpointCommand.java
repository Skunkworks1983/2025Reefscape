// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;

// This command moves to the elevator from its current position to the argument
// targetHeight (meters). This command will end when the elevator is within some
// tolorence of the desired position. This command uses the trapazoid motion profile.
// This command is long and requires a fair amount of state so it is not defined within the
// Elevator subsystem.
public class MoveElevatorToSetpointCommand extends Command {
  Timer timeElapsed;
  State startState;
  State targetState;
  Elevator elevator;
  boolean isGoingUp;
  DoubleSupplier targetHeight;

  private TrapezoidProfile motionProfile ;

  public MoveElevatorToSetpointCommand(Elevator elevator, DoubleSupplier targetHeight, double elevatorVelocityOveride, double elevatorAcceletationOveride) {
    this.targetHeight = targetHeight;
    this.elevator = elevator;
    timeElapsed = new Timer();
    timeElapsed.stop();
    motionProfile = new TrapezoidProfile(
      new Constraints(
        elevatorVelocityOveride,
        elevatorAcceletationOveride
      )
    );

    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    targetState = new State(targetHeight.getAsDouble() * Constants.Elevator.METERS_TO_MOTOR_ROTATIONS, 0.0);
    startState = new State(elevator.getElevatorPosition() * Constants.Elevator.METERS_TO_MOTOR_ROTATIONS, 0.0);
    isGoingUp = targetState.position > elevator.getElevatorPosition() * Constants.Elevator.METERS_TO_MOTOR_ROTATIONS;
    elevator.setFinalPosition(targetState.position);
    timeElapsed.reset();
    timeElapsed.start();
    System.out.println("Move elevator to Setpoint Command Initialize");
  }

  @Override
  public void execute() {

    State newState = motionProfile.calculate(
      timeElapsed.get(), // Time is the only variable that changes throughout each run
      startState, 
      targetState
    );

    elevator.setMotorTrapezoidProfileSafe(newState.position, newState.velocity);
  }

  @Override
  public void end(boolean interrupted) {
    if (elevator.getBottomLimitSwitch() && !isGoingUp) {
      elevator.setMotorTrapezoidProfileSafe(0.0, 0.0);
    }
    System.out.println("Move elevator to Setpoint Command End");
  }

  @Override
  public boolean isFinished() {
    return (Constants.Elevator.TOLERENCE_METERS_FOR_MOVE_TO_POSITION > 
      Math.abs(targetState.position - elevator.getElevatorPosition())) ||
      (
        (elevator.getBottomLimitSwitch() && !isGoingUp)
        // || (elevator.getTopLimitSwitch() && isGoingUp)
      );
  }
}
