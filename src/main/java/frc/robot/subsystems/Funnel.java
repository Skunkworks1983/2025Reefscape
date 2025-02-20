// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.utils.SmartPIDControllerCANSparkMax;

public class Funnel extends SubsystemBase {

  SparkMax pivotMotor;

  private SmartPIDControllerCANSparkMax pivotMotorSpeedController;

  double setPoint;

  public Funnel(/*dont put a peram, not needed  */) {
    //this.pivotMotor = new SparkMax(pivotMotorId, MotorType.kBrushless);
    pivotMotor = new SparkMax(Constants.Funnel.PIVOT_MOTOR_ID,  MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop
      .p(Constants.Funnel.FUNNEL_KP)
      .i(Constants.Funnel.FUNNEL_KI)
      .d(Constants.Funnel.FUNNEL_KD);

    pivotMotorSpeedController = new SmartPIDControllerCANSparkMax(
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
  }

  public double getPos(){
    return pivotMotor.getEncoder().getPosition();
  }

  public double getSetPoint(){
    return setPoint;
  }

  public boolean approxEquals(double value1, double value2, double tolerance) {
    return Math.abs(value1 - value2) < tolerance;
  }

  public boolean isAtSetpoint() {
    return approxEquals(getPos(), getSetPoint(), Constants.ClimberIDs.CLIMBER_TOLERANCE); 
  }

  public void setFunnelSetPoint(double newSetPoint){
    setPoint = getPos() - newSetPoint;
    SparkClosedLoopController FunnelLoopController = pivotMotor.getClosedLoopController();
    FunnelLoopController.setReference(setPoint, ControlType.kPosition);
  }

  public Command GoToPos(double position) {
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
}
