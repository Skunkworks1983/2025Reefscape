// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
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

  private DigitalInput topMagnetSensor;
  private DigitalInput bottomMagnetSensor;

  public Wrist() {
    wristMotor = new TalonFX(Constants.WristIDs.WRIST_KRAKEN_MOTOR_ID);
    wristMotor.setPosition(0.0);

    topMagnetSensor = new DigitalInput(Constants.WristIDs.WRIST_TOP_MAGNET_SENSOR);
    bottomMagnetSensor = new DigitalInput(Constants.WristIDs.WRIST_BOTTOM_MAGNET_SENSOR);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("wrist velocity", getWristVelocity());
    SmartDashboard.putNumber("wrist motor position ", getPosition());

    System.out.println("top magnet " + topMagnetSensor.get());
    System.out.println("bottom magnet "+ bottomMagnetSensor.get());

  }

  public boolean getTopMagnetSensor() { 
    return !topMagnetSensor.get();
  }

  public boolean getBottomMagnetSensor() {
    return !bottomMagnetSensor.get();
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

  public void setWristMotorPosition(double setWristMotorPosition) {
    wristMotor.setPosition(setWristMotorPosition);
  }


}
