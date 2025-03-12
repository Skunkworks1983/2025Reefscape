// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CurrentLimits;
import frc.robot.utils.ConditionalSmartDashboard;
import frc.robot.utils.PIDControllers.SmartPIDController;
import frc.robot.utils.PIDControllers.SmartPIDControllerTalonFX;

public class Wrist extends SubsystemBase {
  TalonFX wristMotor;
  StatusSignal<Angle> wristPos;

  // We do not currently have working limit switches on the wrist
  private DigitalInput topMagnetSensor = new DigitalInput(Constants.Wrist.IDs.WRIST_TOP_MAGNET_SENSOR);
  private DigitalInput bottomMagnetSensor = new DigitalInput(Constants.Wrist.IDs.WRIST_BOTTOM_MAGNET_SENSOR);

  private SmartPIDControllerTalonFX smartPIDController;

  public Wrist() {
    //setDefaultCommand(holdPositionCommand());
    wristMotor = new TalonFX(Constants.Wrist.IDs.WRIST_KRAKEN_MOTOR_ID, "Collector 2025");
    
    wristMotor.setPosition(0.0);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits = CurrentLimits.KRAKEN_CURRENT_LIMIT_CONFIG;
    wristMotor.getConfigurator().apply(config);
    wristMotor.setNeutralMode(NeutralModeValue.Brake);

    smartPIDController = new SmartPIDControllerTalonFX(
      Constants.Wrist.PIDs.WRIST_KP,
      Constants.Wrist.PIDs.WRIST_KI,
      Constants.Wrist.PIDs.WRIST_KD,
      Constants.Wrist.PIDs.WRIST_KF,
      Constants.Wrist.PIDs.WRIST_KV,
      Constants.Wrist.PIDs.WRIST_KA,
      Constants.Wrist.PIDs.WRIST_KS,
      "Wrist", 
      Constants.Wrist.PIDs.WRIST_SMARTPID_ACTIVE, 
      wristMotor
    );
  }

  @Override
  public void periodic() {
    
    ConditionalSmartDashboard.putNumber("Wrist/Wrist velocity (rotations per second)", getWristVelocity());
    SmartDashboard.putNumber("Wrist/Wrist motor position(rotations)", getPosition());
    ConditionalSmartDashboard.putBoolean("Wrist/Bottom wrist magnet state", getBottomMagnetSensor());
    ConditionalSmartDashboard.putBoolean("Wrist/Top wrist magnet state", getTopMagnetSensor());

    // Setposition counts as a config update, try and do this sparingly
    // if (getTopMagnetSensor() && Math.abs(wristMotor.getPosition().getValueAsDouble() - Constants.Wrist.WRIST_MAX_ROTATIONS) > .001) {
    //   wristMotor.setPosition(Constants.Wrist.WRIST_MAX_ROTATIONS);
    // }

    // if (getBottomMagnetSensor() && Math.abs(wristMotor.getPosition().getValueAsDouble() - Constants.Wrist.WRIST_MIN_ROTATIONS) > .001) {
    //   wristMotor.setPosition(Constants.Wrist.WRIST_MIN_ROTATIONS);
    // }
  }

  // magnet outputs reversed so that they are true when triggered
  public boolean getTopMagnetSensor() { 
    return !topMagnetSensor.get();
  }

  public boolean getBottomMagnetSensor() {
    return !bottomMagnetSensor.get();
  }

  public double getPosition() {
    return wristMotor.getPosition().getValueAsDouble() / Constants.Wrist.WRIST_GEAR_RATIO;
  }

  public double getWristVelocity() {
    return wristMotor.getVelocity().getValueAsDouble() / Constants.Wrist.WRIST_GEAR_RATIO;
  }
  
  public void setWristMotorControl(PositionVoltage setWristMotorControl) {
    wristMotor.setControl(setWristMotorControl
      //.withLimitForwardMotion(getTopMagnetSensor())
      //.withLimitReverseMotion(getBottomMagnetSensor())
      .withEnableFOC(true)
    );
  }
  
  public void setWristMotorSpeed(double setWristMotorSpeed) {
    wristMotor.set(setWristMotorSpeed);
  } 

}
