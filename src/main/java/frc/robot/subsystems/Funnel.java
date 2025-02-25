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
import frc.robot.utils.ConditionalSmartDashboard;
import frc.robot.utils.PIDControllers.SmartPIDController;
import frc.robot.utils.PIDControllers.SmartPIDControllerCANSparkMax;
import frc.robot.utils.error.DiagnosticSubsystem;
import frc.robot.utils.error.ErrorGroup;
import frc.robot.utils.error.TestResult;

public class Funnel extends SubsystemBase implements DiagnosticSubsystem{

  SparkMax pivotMotor;

  private SmartPIDControllerCANSparkMax pivotMotorSpeedController;

  double setpoint;

  public Funnel() {
    pivotMotor = new SparkMax(Constants.Funnel.PIVOT_MOTOR_ID,  MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop
      .p(Constants.Funnel.FUNNEL_KP)
      .i(Constants.Funnel.FUNNEL_KI)
      .d(Constants.Funnel.FUNNEL_KD);
    pivotMotor.getEncoder().setPosition(0);

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
    ConditionalSmartDashboard.putNumber("Funnel Pos (revs)", getPos());
  }

  public double getPos(){
    return pivotMotor.getEncoder().getPosition();
  }

  public double getSetpoint(){
    return (setpoint / 360) / Constants.Funnel.PIVOT_MOTOR_GEAR_RATIO;
  }

  public boolean isMotorConnected(){
    return pivotMotor.getFirmwareVersion() != 0;
  }

  public double getCurrent(){
    return pivotMotor.getOutputCurrent();
  }

  public boolean approxEquals(double value1, double value2, double tolerance) {
    return Math.abs(value1 - value2) < tolerance;
  }

  public boolean isAtSetpoint() {
    return approxEquals(getPos(), getSetpoint(), Constants.ClimberIDs.CLIMBER_TOLERANCE); 
  }

  public double getVelocity(){
    return pivotMotor.getEncoder().getVelocity();
  }

  public void setFunnelSetPoint(double revs){
    setpoint = getPos() - revs;
    ConditionalSmartDashboard.putNumber("set point (revs)", setpoint);
    SparkClosedLoopController FunnelLoopController = pivotMotor.getClosedLoopController();
    FunnelLoopController.setReference(getSetpoint(), ControlType.kPosition);
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
