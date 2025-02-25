// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Funnel;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Funnel;

public class MoveToPosition extends Command {
  Funnel funnel;

  PositionVoltage positionVoltage;
  TrapezoidProfile.State goal;
  TrapezoidProfile.State startPosition;
  double setPoint;
  double newSetPoint;

  final TrapezoidProfile profile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(1, 1));
  
  Timer timePassed;

  public MoveToPosition(Funnel funnel, double setPoint) {
    this.funnel = funnel;
    this.setPoint = setPoint;

    addRequirements(funnel);

    timePassed = new Timer();
    timePassed.stop();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("INITILIAZED FUNNEL MOVE TO POS COMMAND");
    timePassed.reset(); 
    timePassed.start();

    goal = new TrapezoidProfile.State(setPoint,0);
    positionVoltage = new PositionVoltage(0);
    startPosition = new TrapezoidProfile.State(funnel.getPos(), funnel.getVelocity());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    State positionGoal = profile.calculate(timePassed.get(), startPosition, goal);
    funnel.setFunnelSetPoint(positionGoal.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("END OF COMMAND");
  }

  @Override
  public boolean isFinished() {
    return funnel.isAtSetpoint();
  }
}
