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
  TalonFX climbMotor;
  StatusSignal<Angle> climbPos;

  private DigitalInput magnetSensor1;
  private DigitalInput magnetSensor2;

  private SmartPIDControllerTalonFX climberSmartPID;
  private PositionVoltage positionVoltage = new PositionVoltage(0);
  private double climberSetPoint = Constants.Climber.CLIMBER_MIN;

  public Climber() {
    climbMotor = new TalonFX(Constants.Climber.IDs.CLIMBER_KRAKEN_MOTOR);
    climbMotor.setPosition(0.0);

    magnetSensor1 = new DigitalInput(Constants.Climber.IDs.CLIMBER_MAGNET_SENSOR_1);
    magnetSensor2 = new DigitalInput(Constants.Climber.IDs.CLIMBER_MAGNET_SENSOR_2);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits = CurrentLimits.MINI_KRAKEN_CURRENT_LIMIT_CONFIG;
    climbMotor.getConfigurator().apply(config);

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
    ConditionalSmartDashboard.putNumber("Climber/Motor position", getHeight());
    ConditionalSmartDashboard.putBoolean("Climber/Motor Connected", isMotorConnected());
    ConditionalSmartDashboard.putNumber("Climber/Motor Current", getCimbMotorCurrent());
    ConditionalSmartDashboard.putBoolean("Climber/Magnet Sensor 1", magnetSensor1Tripped());
    ConditionalSmartDashboard.putBoolean("Climber/Magnet Sensor 2", magnetSensor2Tripped());
    ConditionalSmartDashboard.putNumber("Climber/Set Point", getSetPoint());
    ConditionalSmartDashboard.putBoolean("Climber/At Set Point", isAtSetpoint());
  }

  public boolean magnetSensor1Tripped() {
    return !magnetSensor1.get();
  }

  public boolean magnetSensor2Tripped() {
    return !magnetSensor2.get();
  }

  // returns in meters
  public double getHeight() {
    return climbMotor.getPosition().getValueAsDouble() * Constants.Climber.CLIMBER_MOTOR_ROTATIONS_TO_CLIMBER_HEIGHT;
  }

  public boolean isMotorConnected() {
    return climbMotor.isConnected();
  }

  public void setClimberSetPoint(double newSetPoint) {
    climberSetPoint = newSetPoint;
    climbMotor.setControl(
        positionVoltage.withPosition(newSetPoint / Constants.Climber.CLIMBER_MOTOR_ROTATIONS_TO_CLIMBER_HEIGHT).withEnableFOC(true));
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
    return approxEquals(getHeight(), climberSetPoint, Constants.Climber.CLIMBER_TOLERANCE); 
  }
    
  public double getCimbMotorCurrent() {
    return climbMotor.getSupplyCurrent().getValueAsDouble();
  }

  public Command goToPositionAfterMagnetSensor(double position) {
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
        return magnetSensor1Tripped() && magnetSensor2Tripped();
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