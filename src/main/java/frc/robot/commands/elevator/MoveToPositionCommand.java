// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Elevator.Profile;
import frc.robot.subsystems.Elevator;

// This command moves to the elevator from its current position to the argument
// targetHeight (meters). This command will end when the elevator is within some
// tolorence of the desired position. This command uses the trapazoid motion profile.
// This command is long and requires a fair amount of state so it is not defined within the
// Elevator subsystem.
public class MoveToPositionCommand extends Command {
  Timer timeElapsed;
  State startState;
  State targetState;
  Elevator elevator;

  private final static TrapezoidProfile motionProfile = new TrapezoidProfile(
    new Constraints(
      Profile.MAX_VELOCITY,
      Profile.MAX_ACCELERATION
    )
  );

  public MoveToPositionCommand(Elevator elevator, double targetHeight) {
    this.targetState = new State(targetHeight, 0.0);
    this.elevator = elevator;
    timeElapsed = new Timer();
    timeElapsed.stop();
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    timeElapsed.start();
  }

  @Override
  public void execute() {
    State motionProfileResult = motionProfile.calculate(
      timeElapsed.get(), // Time is the only variable that changes throughout each run
      startState, 
      targetState
    );

    elevator.setMotor(
      elevator.getPositionController().calculate(
        elevator.getElevatorPosition(),
        motionProfileResult.position
      )
    );
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Constants.Elevator.TOLERENCE_METERS_FOR_MOVE_TO_POSITION > 
      Math.abs(targetState.position - elevator.getElevatorPosition());
  }
}
