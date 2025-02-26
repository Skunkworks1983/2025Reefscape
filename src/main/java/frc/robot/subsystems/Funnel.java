// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import java.util.function.Consumer;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutomatedTests.RunClimberMotorTest;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CurrentLimits;
import frc.robot.utils.PIDControllers.SmartPIDController;
import frc.robot.utils.PIDControllers.SmartPIDControllerCANSparkMax;
import frc.robot.utils.error.DiagnosticSubsystem;
import frc.robot.utils.error.ErrorGroup;
import frc.robot.utils.error.TestResult;

public class Funnel extends SubsystemBase implements DiagnosticSubsystem{

  SparkMax pivotMotor;

  private SmartPIDControllerCANSparkMax pivotMotorSpeedController;

  double setPoint;

  public Funnel() {
    pivotMotor = new SparkMax(Constants.Funnel.PIVOT_MOTOR_ID,  MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config
      .secondaryCurrentLimit(CurrentLimits.NEO_550_CURRENT_LIMIT_VALUE)
      .closedLoop
        .p(Constants.Funnel.FUNNEL_KP)
        .i(Constants.Funnel.FUNNEL_KI)
        .d(Constants.Funnel.FUNNEL_KD);

    pivotMotorSpeedController = new frc.robot.utils.PIDControllers.SmartPIDControllerCANSparkMax(
      Constants.Funnel.FUNNEL_KP,
      Constants.Funnel.FUNNEL_KI,
      Constants.Funnel.FUNNEL_KD,
      Constants.Funnel.FUNNEL_KF,
      "Funnel Pivot Motor",
      Constants.Funnel.FUNNEL_SMARTPID_ACTIVE,
      pivotMotor
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Funnel Pos (revs)", getPos());
  }

  public double getPos(){
    return pivotMotor.getEncoder().getPosition();
  }

  public double getSetPoint(){
    return setPoint;
  }

  public boolean isMotorConnected(){
    return pivotMotor.getFirmwareVersion() != 00;
  }

  public boolean approxEquals(double value1, double value2, double tolerance) {
    return Math.abs(value1 - value2) < tolerance;
  }

  public boolean isAtSetpoint() {
    return approxEquals(getPos(), getSetPoint(), Constants.ClimberIDs.CLIMBER_TOLERANCE); 
  }

  public void setFunnelSetPoint(double revs){
    setPoint = getPos() - revs;
    SmartDashboard.putNumber("set point (revs)", setPoint);
    SparkClosedLoopController FunnelLoopController = pivotMotor.getClosedLoopController();
    FunnelLoopController.setReference(setPoint, ControlType.kPosition);
  }

  public Command goToPos(double position) {
    return Commands.startEnd(
      () -> {
        setFunnelSetPoint(position);
      }, () -> {

      }).until(
        () -> {
          return isAtSetpoint();
        }
      );
  }

  public Command hardwareConnectionTest(Consumer<TestResult> addTest,
  Consumer<TestResult> setTest) {
    TestResult funnelMotorConnectedTest = new TestResult(
      "Funnel Motor is not Connected",
      AlertType.kWarning,
      this,
      "checks if motor is connected"
    );
    addTest.accept(funnelMotorConnectedTest);
    return Commands.startEnd(
        () -> {

        },
        () -> {
          if(isMotorConnected()) {
            funnelMotorConnectedTest.setErrorStatus(AlertType.kError);
          }
          else {
            funnelMotorConnectedTest.setErrorStatus(AlertType.kInfo);
          }
        });
  }

  @Override
  public Command getErrorCommand(ErrorGroup errorGroupHandler) {
    return Commands.sequence(
        hardwareConnectionTest(errorGroupHandler::addTestSetEntry, errorGroupHandler::setTestStatusUsingTestResult)
    );
  }
}
