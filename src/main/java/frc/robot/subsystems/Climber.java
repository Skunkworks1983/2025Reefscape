// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutomatedTests.RunClimberMotorTest;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CurrentLimits;
import frc.robot.utils.ConditionalSmartDashboard;
import frc.robot.utils.PIDControllers.SmartPIDControllerTalonFX;
import frc.robot.utils.error.DiagnosticSubsystem;
import frc.robot.utils.error.ErrorGroup;
import frc.robot.utils.error.TestResult;

public class Climber extends SubsystemBase implements DiagnosticSubsystem {
  // climber motor, Talon FX
  TalonFX climbMotor;

  // magnet sensors for the climber
  private DigitalInput magnetSensor1;
  private DigitalInput magnetSensor2;

  // smart pid code for the climber motor
  private SmartPIDControllerTalonFX climberSmartPID;
  // position voltage for climber
  private PositionVoltage positionVoltage = new PositionVoltage(0);
  // the base climber set point
  private double climberSetPointMeters = Constants.Climber.CLIMBER_MIN;

  public Climber() {
    // instantiates and sets the position of the climber motor
    climbMotor = new TalonFX(Constants.Climber.IDs.CLIMBER_KRAKEN_MOTOR);
    climbMotor.setPosition(0.0);

    // instantiates the climber magnet sensors
    magnetSensor1 = new DigitalInput(Constants.Climber.IDs.CLIMBER_MAGNET_SENSOR_1);
    magnetSensor2 = new DigitalInput(Constants.Climber.IDs.CLIMBER_MAGNET_SENSOR_2);

    // configs for climber
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits = CurrentLimits.MINI_KRAKEN_CURRENT_LIMIT_CONFIG;
    climbMotor.getConfigurator().apply(config);

    //smart pid for climber
    climberSmartPID = new SmartPIDControllerTalonFX(
      Constants.Climber.PIDs.CLIMBER_KP,
      Constants.Climber.PIDs.CLIMBER_KI,
      Constants.Climber.PIDs.CLIMBER_KD,
      Constants.Climber.PIDs.CLIMBER_KF,
      "Climb Motor",
      Constants.Climber.CLIMBER_SMARTPID_ACTIVE,
      climbMotor
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //smartdashboard for climber
    SmartDashboard.putNumber("Climber/Motor position", getHeight());
    ConditionalSmartDashboard.putBoolean("Climber/Motor Connected", isMotorConnected());
    SmartDashboard.putNumber("Climber/Motor Current", getClimberMotorCurrent());
    SmartDashboard.putBoolean("Climber/Magnet Sensor 1", magnetSensor1Tripped());
    SmartDashboard.putBoolean("Climber/Magnet Sensor 2", magnetSensor2Tripped());
    ConditionalSmartDashboard.putNumber("Climber/Set Point", getSetPointMeters());
    ConditionalSmartDashboard.putBoolean("Climber/At Set Point", isAtSetpoint());
  }

  //Checks is first magnet sensor is activated. returns true if activated
  public boolean magnetSensor1Tripped() {
    return !magnetSensor1.get();
  }

  //Checks is second magnet sensor is activated. returns true if activated
  public boolean magnetSensor2Tripped() {
    return !magnetSensor2.get();
  }

  // returns height of climber in meters
  public double getHeight() {
    return climbMotor.getPosition().getValueAsDouble() * Constants.Climber.CLIMBER_MOTOR_ROTATIONS_TO_CLIMBER_HEIGHT;
  }

  // Checks if the motor is connected
  public boolean isMotorConnected() {
    return climbMotor.isConnected();
  }

  // sets the climber to a set point defined in parameter. parameter is in meters.
  public void setClimberSetPointMeters(double newSetPointMeters) {
    climberSetPointMeters = newSetPointMeters;
    climbMotor.setControl(
        positionVoltage.withPosition(newSetPointMeters / Constants.Climber.CLIMBER_MOTOR_ROTATIONS_TO_CLIMBER_HEIGHT).withEnableFOC(true));
    ConditionalSmartDashboard.putNumber("Motor position", getHeight());
  }

  // returns the last set point in meters
  public double getSetPointMeters() {
    ConditionalSmartDashboard.putNumber("climber set point", climberSetPointMeters);
    return climberSetPointMeters;
  }

  // returns true if the difference between the first 2 parameter is less than the last parameter 
  public boolean approxEquals(double value1, double value2, double tolerance) {
    return Math.abs(value1 - value2) < tolerance;
  }

  // returns true if climber is at set point
  public boolean isAtSetpoint() {
    return approxEquals(getHeight(), climberSetPointMeters, Constants.Climber.CLIMBER_TOLERANCE); 
  }
    
  // returns the climber current in amps
  public double getClimberMotorCurrent() {
    return climbMotor.getSupplyCurrent().getValueAsDouble();
  }

  // if the magnet sensors are true, go to parameter (position in meters)
  public Command goToPositionAfterMagnetSensor(double position) {
    return Commands.startEnd(
        () -> {
          //waitUntilMagnetSensorsAreTrue().finallyDo(
                setClimberSetPointMeters(position);
        }, () -> {

        }).until(
            () -> {
              return isAtSetpoint();
            });
  }

  // ends if magnet sensors are true
  public Command waitUntilMagnetSensorsAreTrue() {
    return Commands.waitUntil(
      () -> {
        return magnetSensor1Tripped() && magnetSensor2Tripped();
      }
    );
  }

  // hardware connection test
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
        if (!isMotorConnected()) {
          climbMotorConnectedTest.setErrorStatus(AlertType.kError);
        }
        else {
          climbMotorConnectedTest.setErrorStatus(AlertType.kInfo);
        }
      }
    );
  }

  // get error command
  @Override
  public Command getErrorCommand(ErrorGroup errorGroupHandler) {
    return Commands.sequence(
        hardwareConnectionTest(errorGroupHandler::addTestSetEntry, errorGroupHandler::setTestStatusUsingTestResult),
        new RunClimberMotorTest(errorGroupHandler::addTestSetEntry, errorGroupHandler::setTestStatusUsingTestResult, this));
  }
}