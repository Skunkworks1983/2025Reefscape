// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Collector extends SubsystemBase {

  TalonFX rightMotor; //change to mini krakens
  TalonFX leftMotor; //change to mini krakens

   final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
   double lastRightSpeed;
   double lastLeftSpeed;


  /** Creates a new Collector. */
  private Collector() {
   rightMotor = new TalonFX(Constants.Collector.RIGHT_MOTOR);
    leftMotor = new TalonFX(Constants.Collector.LEFT_MOTOR);
    
    TalonFXConfiguration talonConfigCollectorMotor = new TalonFXConfiguration();

    talonConfigCollectorMotor.MotorOutput.NeutralMode = NeutralModeValue.Brake;

   rightMotor.getConfigurator().apply(talonConfigCollectorMotor);
   leftMotor.getConfigurator().apply(talonConfigCollectorMotor);

   // rightMotor.run();
    
  }

  public void setCollectorSpeeds(double rightSpeed, double leftSpeed){
    if (rightSpeed != lastRightSpeed) {
      rightMotor.setControl(velocityVoltage
          .withVelocity(rightSpeed * Constants.Collector.COLLECTOR_ROTATIONS_PER_METER));
    }
    lastRightSpeed = rightSpeed;

    if (leftSpeed != lastLeftSpeed) {
      leftMotor.setControl(velocityVoltage
          .withVelocity(leftSpeed * Constants.Collector.COLLECTOR_ROTATIONS_PER_METER));
    }
    lastLeftSpeed = leftSpeed;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public Command rotateCoral() {
    return Commands.runEnd(
      () -> {
        setCollectorSpeeds(Constants.Collector.COLLECOR_ROTATE_SLOW, 
        Constants.Collector.COLLECOR_ROTATE_FAST);
      }, 
      () -> {
        setCollectorSpeeds(0, 0);
      }
    ).until(
      () -> {
      return false;
      }
    );
  }

  public Command intakeCoral() {
    return Commands.runEnd(
      () -> {
        setCollectorSpeeds(Constants.Collector.COLLECOR_ROTATE_FAST, Constants.Collector.COLLECOR_ROTATE_FAST);
      },
      () -> {
        setCollectorSpeeds(0, 0);
      }
      );
  }

}
