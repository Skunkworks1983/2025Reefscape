// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.ConditionalSmartDashboard;


public class MoveWristToSetpoint extends Command {
  Wrist wrist;

  PositionVoltage positionVoltage;
  TrapezoidProfile.State goal;
  TrapezoidProfile.State startPosition;
  double setPoint;

  final TrapezoidProfile profile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(1, 1));
  
  Timer timePassed;

  public MoveWristToSetpoint(Wrist wrist, double setPoint) {
    this.setPoint = setPoint;
    this.wrist = wrist;

    addRequirements(wrist);

    timePassed = new Timer();
    timePassed.stop();
  }
  
  @Override
  public void initialize() {
    DataLogManager.log("MoveWristToSetpoint command initialized");

    timePassed.reset(); 
    timePassed.start();

    double circlePortion = setPoint / 360.0;
    double newSetPoint = circlePortion * Constants.Wrist.WRIST_GEAR_RATIO;

    goal = new TrapezoidProfile.State(newSetPoint,0);
    positionVoltage = new PositionVoltage(0);
    startPosition = new TrapezoidProfile.State(wrist.getPosition(),wrist.getWristVelocity());
  }

  @Override
  public void execute() {
    State positionGoal = profile.calculate(timePassed.get(), startPosition, goal);
    positionVoltage.Position = positionGoal.position;
    positionVoltage.Velocity = positionGoal.velocity;
    wrist.setWristMotorControl(positionVoltage);

    ConditionalSmartDashboard.putNumber("Wrist position goal (motor rotations)", positionGoal.position);
    ConditionalSmartDashboard.putNumber("Wrist velocity goal (motor rotations per second)", positionGoal.velocity);
  }
  
  @Override
  public void end(boolean interrupted) {
    wrist.setWristMotorSpeed(0);
    DataLogManager.log("MoveWristToSetpoint command ended");
  }
  
  @Override
  public boolean isFinished() {
    return (Math.abs(wrist.getPosition() - setPoint)) < Constants.Wrist.WRIST_TOLERANCE;
  }
}
