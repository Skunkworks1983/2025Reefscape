// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.utils.SmartPIDControllerTalonFX;

public class Wrist extends SubsystemBase {
  TalonFX wristMotor;
  StatusSignal<Angle> wristPos;

  private DigitalInput magnetSensor1;

  private SmartPIDControllerTalonFX wristSmartPID;

  public Wrist() {
    wristMotor = new TalonFX(Constants.WristIDs.WRIST_KRAKEN_MOTOR_ID);
    wristMotor.setPosition(0.0);

    magnetSensor1 = new DigitalInput(Constants.WristIDs.WRIST_MAGNET_SENSOR_1);
    
    wristSmartPID = new SmartPIDControllerTalonFX(
        Constants.WristIDs.WRIST_KP,
        Constants.WristIDs.WRIST_KI,
        Constants.WristIDs.WRIST_KD,
        Constants.WristIDs.WRIST_KF,
        "Climb Motor",
        Constants.WristIDs.WRIST_SMARTPID_ACTIVE,
        wristMotor
        );
    
    Trigger wristMoveTrigger = new Trigger(() -> this.getMagnetSensor1());
    
    wristMoveTrigger.onFalse(this.MoveWrist());
    
  }

  @Override
  public void periodic() {
    //System.out.println("running");
  }

  public Command MoveWrist() { //TODO add directions
    System.out.println("Is running");
    return moveInDirection(Constants.WristIDs.WRIST_MIDPOINT_ROTATIONS);
  }

  public boolean getMagnetSensor1() {
    //System.out.println(!magnetSensor1.get());
    return !magnetSensor1.get();
    
  }

  public double getPosition() {
    return wristMotor.getPosition().getValueAsDouble();
  }


  

  public Command moveInDirection(double setPoint)
  {
    VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    final double newSetPoint = setPoint + getPosition();
    
    return Commands.runEnd(
      () -> {
        wristSmartPID.updatePID();

        if (Math.abs(getPosition() - newSetPoint) < Constants.WristIDs.WRIST_RANGE) {
          return;
        }
        
          wristMotor.setControl(velocityVoltage.withVelocity(Constants.WristIDs.WRIST_VELOCITY));
        

        SmartDashboard.putNumber("wrist motor position: ", getPosition());
        SmartDashboard.putNumber("wrist motor set point: ", newSetPoint);
        SmartDashboard.putNumber("stop condition", Math.abs(getPosition() - newSetPoint));

        
      },
      () -> {
        wristMotor.stopMotor();
      }
    ).until(() -> Math.abs(getPosition() - newSetPoint) < Constants.WristIDs.WRIST_RANGE || getMagnetSensor1());
  }

  
}
