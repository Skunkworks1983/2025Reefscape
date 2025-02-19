// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Wrist;


public class MoveInDirection extends Command {
  Wrist wrist;

  PositionVoltage positionVoltage;
  TrapezoidProfile.State m_goal;
  TrapezoidProfile.State m_setpoint;
  double setPoint;
  double newSetPoint;
  final TrapezoidProfile m_profile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(1, 1));
  
  public MoveInDirection(Wrist wrist, double setPoint) {
    this.setPoint = setPoint;
    this.wrist = wrist;

    addRequirements(wrist);
  }
  
  @Override
  public void initialize() {
    newSetPoint = setPoint + wrist.getPosition();

    m_goal = new TrapezoidProfile.State(newSetPoint,0);
    positionVoltage = new PositionVoltage(0);
    m_setpoint = new TrapezoidProfile.State();
    
  }



  
  @Override
  public void execute() {
    m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);

    positionVoltage.Position = m_setpoint.position;
    positionVoltage.Velocity = m_setpoint.velocity;
    wrist.setWristMotorControl(positionVoltage);

    System.out.println("is running");
  }




  
  @Override
  public void end(boolean interrupted) {
    wrist.setWristMotorSpeed(0);
  }

  
  @Override
  public boolean isFinished() {
    if (Math.abs(wrist.getPosition() - newSetPoint) < Constants.WristIDs.WRIST_RANGE || wrist.getMagnetSensor1()) {
      return true;
    }
    return false;
  }
}
