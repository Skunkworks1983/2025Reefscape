// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

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
public class MoveToPositionCommand extends Command {
  State startState;
  State targetState;
  Elevator elevator;

  private final static TrapezoidProfile motionProfile = new TrapezoidProfile(
    new Constraints(
      Constants.Elevator.Profile.MAX_VELOCITY,
      Constants.Elevator.Profile.MAX_ACCELERATION
    )
  );

  public MoveToPositionCommand(Elevator elevator, double targetHeight) {
    this.targetState = new State(targetHeight * Constants.Elevator.METERS_TO_MOTOR_ROTATIONS, 0.0);
    startState = new State(elevator.getElevatorPosition() * Constants.Elevator.METERS_TO_MOTOR_ROTATIONS, 0.0);
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    startState = motionProfile.calculate(
      0.020,
      startState, 
      targetState
    );
    
    elevator.setMotorTrapezoidProfileSafe(startState.position, startState.velocity);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Constants.Elevator.TOLERENCE_METERS_FOR_MOVE_TO_POSITION > 
      Math.abs(targetState.position - elevator.getElevatorPosition());
  }
}
