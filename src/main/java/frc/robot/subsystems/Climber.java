// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutomatedTests.RunClimberMotorTest;
import frc.robot.constants.Constants;
import frc.robot.utils.ConditionalSmartDashboard;
import frc.robot.utils.PIDControllers.SmartPIDControllerTalonFX;
import frc.robot.utils.error.DiagnosticSubsystem;
import frc.robot.utils.error.ErrorGroup;
import frc.robot.utils.error.TestResult;

public class Climber extends SubsystemBase implements DiagnosticSubsystem {
  TalonFX climbMotor;
  StatusSignal<Angle> climbPos;

  private DigitalInput magnetSensor1;
  private DigitalInput magnetSensor2;

  private SmartPIDControllerTalonFX climberSmartPID;
  private PositionVoltage positionVoltage = new PositionVoltage(0);
  private double climberSetPoint = Constants.ClimberIDs.CLIMBER_MIN;

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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber/Motor position", getHeight());
    SmartDashboard.putBoolean("Climber/Motor Connected", isMotorConnected());
    SmartDashboard.putNumber("Climber/Motor Current", getCurrent());
    SmartDashboard.putBoolean("Climber/Magnet Sensor 1", getMagnetSensor1());
    SmartDashboard.putBoolean("Climber/Magnet Sensor 2", getMagnetSensor2());
    SmartDashboard.putNumber("Climber/Set Point", getSetPoint());
    SmartDashboard.putBoolean("Climber/At Set Point", isAtSetpoint());
  }

  //Negated because magnet sensors return false when activated even though there supposed to be true when activated 
  //and when there true the command activates sooooooo...
  public boolean getMagnetSensor1() {
    return !magnetSensor1.get();
  }

  public boolean getMagnetSensor2() {
    return !magnetSensor2.get();
  }

  // returns in meters
  public double getHeight() {
    return climbMotor.getPosition().getValueAsDouble() * Constants.ClimberIDs.CLIMBER_MOTOR_ROTATIONS_TO_CLIMBER_HEIGHT;
  }

  public boolean isMotorConnected() {
    return climbMotor.isConnected();
  }

  public void setClimberSetPoint(double newSetPoint) {
    climberSetPoint = newSetPoint;
    climbMotor.setControl(
        positionVoltage.withPosition(newSetPoint / Constants.ClimberIDs.CLIMBER_MOTOR_ROTATIONS_TO_CLIMBER_HEIGHT));
    ConditionalSmartDashboard.putNumber("Motor position", getHeight());
  }

  public double getSetPoint() {
    ConditionalSmartDashboard.putNumber("climber set point", climberSetPoint);
    return climberSetPoint;
  }

  public boolean approxEquals(double value1, double value2, double tolerance) {
    return Math.abs(value1 - value2) < tolerance;
  }

  public boolean isAtSetpoint() {
    return approxEquals(getHeight(), climberSetPoint, Constants.ClimberIDs.CLIMBER_TOLERANCE); 
  }
    
  public double getCurrent() {
    return climbMotor.getSupplyCurrent().getValueAsDouble();
  }

  public Command waitUntilMagnetSensorsAreTrueThenGoToPos(double position) {
    return Commands.startEnd(
        () -> {
          waitUntilMagnetSensorsAreTrue().finallyDo(
              () -> {
                setClimberSetPoint(position);
              }).schedule();

        }, () -> {

        }).until(
            () -> {
              return isAtSetpoint();
            });
  }

  public Command waitUntilMagnetSensorsAreTrue() {
    return Commands.waitUntil(
      () -> {
        return getMagnetSensor1() && getMagnetSensor2();
      }
    );
  }

  public Command hardwareConnectionTest(
    Consumer<TestResult> addTest,
    Consumer<TestResult> setTest
  ) {

    TestResult climbMotorConnectedTest = new TestResult(
      "Climb Motor is not Connected",
      AlertType.kWarning,
      this,
      "checks if motor is connected"
    );
    addTest.accept(climbMotorConnectedTest);

    return Commands.startEnd(
      () -> {
        
      },
      () -> {
        if(!isMotorConnected()) {
          climbMotorConnectedTest.setErrorStatus(AlertType.kError);
        }
        else {
          climbMotorConnectedTest.setErrorStatus(AlertType.kInfo);
        }
      }
    );
  }

  @Override
  public Command getErrorCommand(ErrorGroup errorGroupHandler) {
    return Commands.sequence(
        hardwareConnectionTest(errorGroupHandler::addTestSetEntry, errorGroupHandler::setTestStatusUsingTestResult),
        new RunClimberMotorTest(errorGroupHandler::addTestSetEntry, errorGroupHandler::setTestStatusUsingTestResult, this));
  }
}