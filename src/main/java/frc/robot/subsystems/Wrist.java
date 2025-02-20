// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Wrist extends SubsystemBase {
  TalonFX wristMotor;
  StatusSignal<Angle> wristPos;

  private DigitalInput magnetSensor1;

  public Wrist() {
    wristMotor = new TalonFX(Constants.WristIDs.WRIST_KRAKEN_MOTOR_ID);
    wristMotor.setPosition(0.0);

    magnetSensor1 = new DigitalInput(Constants.WristIDs.WRIST_MAGNET_SENSOR_1);
    
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.0; 
    slot0Configs.kV = 0.12; 
    slot0Configs.kP = 3; 
    slot0Configs.kI = 0; 
    slot0Configs.kD = 0.1;
  }

  @Override
  public void periodic() {
    resetWhenMagnetTriggered();

    SmartDashboard.putNumber("wrist velocity", getWristVelocity());
    SmartDashboard.putNumber("wrist motor position ", getPosition());
  }

  public void resetWhenMagnetTriggered() {
    if (getMagnetSensor1()) {
      wristMotor.setPosition(0.0);
    }
  }

  public boolean getMagnetSensor1() {
    return !magnetSensor1.get();
  }

  public double getPosition() {
    return wristMotor.getPosition().getValueAsDouble();
  }

  public double getWristVelocity() {
    return wristMotor.getVelocity().getValueAsDouble();
  }
  
  public void setWristMotorControl(PositionVoltage setWristMotorSpeed) {
    wristMotor.setControl(setWristMotorSpeed);
  }
  
  public void setWristMotorSpeed(double setWristMotorSpeed) {
    wristMotor.set(setWristMotorSpeed);
  }

/* 
  public Command moveInDirection(double setPoint)
  {
    PositionVoltage positionVoltage = new PositionVoltage(0);
    
    final double newSetPoint = setPoint + getPosition();
    final TrapezoidProfile m_profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(1, 1));
    
    TrapezoidProfile.State m_goal = new TrapezoidProfile.State(newSetPoint,0);
    TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

     return Commands.runEnd(
      () -> {
        m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);

        positionVoltage.Position = m_setpoint.position;
        positionVoltage.Velocity = m_setpoint.velocity;
        wristMotor.setControl(positionVoltage);  

        if (Math.abs(getPosition() - newSetPoint) < Constants.WristIDs.WRIST_RANGE) {
          return;
        }

        SmartDashboard.putNumber("wrist motor position ", getPosition());
        SmartDashboard.putNumber("wrist motor set point ", newSetPoint);
        SmartDashboard.putNumber("stop condition", Math.abs(getPosition() - newSetPoint));

      },
      () -> {
        wristMotor.stopMotor();
      },
      this
    ).until(() -> Math.abs(getPosition() - newSetPoint) < Constants.WristIDs.WRIST_RANGE || getMagnetSensor1()); */
  

  
}
