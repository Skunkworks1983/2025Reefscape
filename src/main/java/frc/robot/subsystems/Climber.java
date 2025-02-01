// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
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
    climbPos = climbMotor.getPosition();

    // instantiates magnet senors
    magnetSensor1 = new DigitalInput(Constants.ClimberIDs.CLIMBER_MAGNET_SENSOR_1);
    magnetSensor2 = new DigitalInput(Constants.ClimberIDs.CLIMBER_MAGNET_SENSOR_2);

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
    return climbPos.getValueAsDouble();
  }

  public void moveInDirection(double setPoint) {

    while (climbMotor.getPosition().getValueAsDouble() != setPoint) {
      climbMotor.set(0.4);
      SmartDashboard.putNumber("climber KP: ", 0.0);
      SmartDashboard.putNumber("climber KD: ", 0.0);
      SmartDashboard.putNumber("climber KI: ", 0.0);
      SmartDashboard.putNumber("climber KF: ", 0.0);
      climberSmartPID.updatePID();
      if (climbMotor.getPosition().getValueAsDouble() == setPoint) {
        climbMotor.stopMotor();
        break;
      }
    }

  }

  public void move(direction myVar) {

    switch (myVar) {
      // when i ask it to go up, it go up
      case UP:
        System.out.println("TBD");
        moveInDirection(Constants.ClimberIDs.CLIMBER_MAX);
        break;

      // when i ask it to stay, it stay
      case STATIONARY:
        System.out.println("TBD");
        break;

      // when i ask it to go down, it go down
      case DOWN:
        System.out.println("TBD");
        climbMotor.set(-.01);
        break;
    }

  }

  public Command moveUP() {
    direction UP = direction.UP;

    return Commands.runOnce(
      () -> {
        move(UP);
      }
    );
  }
}
