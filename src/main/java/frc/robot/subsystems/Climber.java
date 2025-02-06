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

public class Climber extends SubsystemBase {
  TalonFX climbMotor;
  StatusSignal<Angle> climbPos;

  enum direction {
    UP,
    STATIONARY,
    DOWN
  }

  private DigitalInput magnetSensor1;
  private DigitalInput magnetSensor2;

  private SmartPIDControllerTalonFX climberSmartPID;

  public Climber() {
    climbMotor = new TalonFX(Constants.ClimberIDs.CLIMBER_KRAKEN_MOTOR);
    climbMotor.setPosition(0.0);

    magnetSensor1 = new DigitalInput(Constants.ClimberIDs.CLIMBER_MAGNET_SENSOR_1);
    magnetSensor2 = new DigitalInput(Constants.ClimberIDs.CLIMBER_MAGNET_SENSOR_2);

    climberSmartPID = new SmartPIDControllerTalonFX(
        Constants.ClimberIDs.CLIMBER_KP,
        Constants.ClimberIDs.CLIMBER_KI,
        Constants.ClimberIDs.CLIMBER_KD,
        Constants.ClimberIDs.CLIMBER_KF,
        "Climb Motor",
        Constants.ClimberIDs.CLIMBER_SMARTPID_ACTIVE,
        climbMotor
        );

    direction up = direction.UP;
    direction stationary = direction.STATIONARY;
    direction down = direction.DOWN;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getMagnetSensor1() {
    return magnetSensor1.get();
  }

  public boolean getMagnetSensor2() {
    return magnetSensor2.get();
  }

  public double getPosition() {
    return climbMotor.getPosition().getValueAsDouble();
  }


  public Command move(direction myDirection) {

    return Commands.runOnce(
      () -> {
        switch (myDirection) {
          // when i ask it to go up, it go up
          case UP:
            checkMagnetSensors();
            moveInDirection(Constants.ClimberIDs.CLIMBER_MAX);
            break;
    
          // when i ask it to stay, it stay
          case STATIONARY:
            checkMagnetSensors();
            moveInDirection(0.0);
            break;
    
          // when i ask it to go down, it go down
          case DOWN:
            checkMagnetSensors();
            moveInDirection(Constants.ClimberIDs.CLIMBER_MIN);
            break;
        }
      }
    );
    

  }

  public Command checkMagnetSensors() {

    return Commands.startEnd(
      () -> {
      },
      () -> {
      }
      ).until(
      () -> {
          if (getMagnetSensor1() && getMagnetSensor2()) {
              return true;
          } else {
              return false;
          }
        }
      );
  }

  public Command moveInDirection(double setPoint)
  {
    VelocityVoltage VV = new VelocityVoltage(0);
    final double newSetPoint = setPoint + getPosition();
    return Commands.runEnd(
      () -> {
        climberSmartPID.updatePID();

        climbMotor.setControl(VV.withVelocity(5)); // TODO figure out velocity

        System.out.println("Position: " + getPosition());
        System.out.println("set Point: " + newSetPoint);
        SmartDashboard.putNumber("position", getPosition());
        SmartDashboard.putNumber("set point", newSetPoint);
      },
      () -> {
        climbMotor.stopMotor();
      }
    ).until(() -> getPosition() > newSetPoint);
  }

  
}
