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
import frc.robot.constants.Constants;
import frc.robot.utils.SmartPIDControllerTalonFX;

public class Wrist extends SubsystemBase {
  TalonFX wristMotor;
  StatusSignal<Angle> wristPos;

  enum direction {
    UP,
    STATIONARY,
    DOWN
  }

  private DigitalInput magnetSensor1;

  private SmartPIDControllerTalonFX wristSmartPID;

  public Wrist() {
    wristMotor = new TalonFX(Constants.WristIDs.WRIST_KRAKEN_MOTOR_ID);
    wristMotor.setPosition(0.0);

    magnetSensor1 = new DigitalInput(Constants.WristIDs.WRIST_MAGNET_SENSOR_1);
    
    /*wristSmartPID = new SmartPIDControllerTalonFX(
        Constants.WristIDs.WRIST_KP,
        Constants.WristIDs.WRIST_KI,
        Constants.WristIDs.WRIST_KD,
        Constants.WristIDs.WRIST_KF,
        "Climb Motor",
        Constants.WristIDs.WRIST_SMARTPID_ACTIVE,
        wristMotor
        );
        */
 
  }

  @Override
  public void periodic() {
    //System.out.println("running");
    // This method will be called once per scheduler run
    //getMagnetSensor1();
    wristMotor.set(0.5);


    if (getMagnetSensor1()) {
      wristMotor.set(0.1);
      System.out.println("setting speed 0.5");
    }
    else {
      wristMotor.set(0);
    }
      
  }

  public boolean getMagnetSensor1() {
    System.out.println(!magnetSensor1.get());
    return !magnetSensor1.get();
    
  }

  public double getPosition() {
    return wristMotor.getPosition().getValueAsDouble();
  }


  public Command waitUntilMagnetSensorsAreTrueThenMove(direction myDirection) {
    return null;
  }

  public Command waitUntilMagnetSensorsAreTrue() {
    return null;
  }
  

  public Command moveInDirection(double setPoint)
  {
    VelocityVoltage VV = new VelocityVoltage(0);
    final double newSetPoint = setPoint + getPosition();
    return Commands.runEnd(
      () -> {
        wristSmartPID.updatePID();

        wristMotor.setControl(VV.withVelocity(Constants.WristIDs.WRIST_VELOCITY));

        SmartDashboard.putNumber("position of wrist motor", getPosition());
        SmartDashboard.putNumber("set point of wrist", newSetPoint);
      },
      () -> {
        wristMotor.stopMotor();
      }
    ).until(() -> getPosition() > newSetPoint + Constants.WristIDs.WRIST_RANGE && 
      getPosition() < newSetPoint - Constants.WristIDs.WRIST_RANGE);
  }

  
}
