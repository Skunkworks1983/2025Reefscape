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
  /** Creates a new Climber. */
  TalonFX climbMotor;
  StatusSignal<Angle> climbPos;

  enum direction {
    UP,
    STATIONARY,
    DOWN
  }

  private DigitalInput magnetSensor1;
  private DigitalInput magnetSensor2;

  SmartPIDControllerTalonFX climberSmartPID;

  public Climber() {
    // instantiates climb motor
    climbMotor = new TalonFX(Constants.ClimberIDs.CLIMBER_KRAKEN_MOTOR);
    climbMotor.setPosition(0.0);

    // instantiates magnet senors
    magnetSensor1 = new DigitalInput(Constants.ClimberIDs.CLIMBER_MAGNET_SENSOR_1);
    // magnetSensor2 = new
    // DigitalInput(Constants.ClimberIDs.CLIMBER_MAGNET_SENSOR_2);

    // instantiates PID
    climberSmartPID = new SmartPIDControllerTalonFX(Constants.ClimberIDs.CLIMBER_KP,
        Constants.ClimberIDs.CLIMBER_KI,
        Constants.ClimberIDs.CLIMBER_KD,
        Constants.ClimberIDs.CLIMBER_KF,
        "Climb Motor",
        Constants.ClimberIDs.CLIMBER_SMARTPID_ACTIVE,
        climbMotor);

    // instantiates enum
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

  public void moveInDirection(double setPoint) {

    setPoint += getPosition();

    while (getPosition() != setPoint) {

      VelocityVoltage VV = new VelocityVoltage(5);
      climbMotor.setControl(VV);
      SmartDashboard.putNumber("climber KP: ", 0.0);
      SmartDashboard.putNumber("climber KD: ", 0.0);
      SmartDashboard.putNumber("climber KI: ", 0.0);
      SmartDashboard.putNumber("climber KF: ", 0.0);

      System.out.println("Position: " + getPosition());
      System.out.println("set Point: " + setPoint);
      SmartDashboard.putNumber("position", getPosition());
      SmartDashboard.putNumber("set point", setPoint);

      climberSmartPID.updatePID();

      if (getPosition() > setPoint) {
        climbMotor.stopMotor();
        break;
      }
    }

  }

  public void move(direction myDirection) {

    switch (myDirection) {
      // when i ask it to go up, it go up
      case UP:
        moveInDirection(Constants.ClimberIDs.CLIMBER_MAX);
        break;

      // when i ask it to stay, it stay
      case STATIONARY:
        moveInDirection(0.0);
        break;

      // when i ask it to go down, it go down
      case DOWN:
        moveInDirection(Constants.ClimberIDs.CLIMBER_MIN);
        break;
    }

  }

  public Command checkMagnetSensors() {

    return Commands.startEnd(
        () -> {

        },
        () -> {

        }).until(
            () -> {
              if (getMagnetSensor1() != true && getMagnetSensor2() != true) {
                return false;
              } else {
                return true;
              }
            });
  }

  public Command moveUP() {
    direction UP = direction.UP; // instantiates the UP direction

    // runs a command to make it go up
    return Commands.runOnce(
        () -> {
          checkMagnetSensors();
          move(UP);
        });
  }

  public Command moveDOWN() {
    direction DOWN = direction.DOWN; // instantiates the DOWN direction

    // runs a command to make it go down
    return Commands.runOnce(
        () -> {
          checkMagnetSensors();
          move(DOWN);
        });
  }

  public Command stayInPlace() {
    direction STATIONARY = direction.STATIONARY; // instantiates the STATIONARY direction

    // runs a command to make it stay where it is
    return Commands.runOnce(
        () -> {
          checkMagnetSensors();
          move(STATIONARY);
        });
  }
}
