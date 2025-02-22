// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.utils.PIDControllers.SmartPIDController;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


// All positions are stored in meters, all velocities are in meters/seconds, 
// all accelerations are stored in meters/second/second.
public class Elevator extends SubsystemBase {

  private TalonFX motorRight = new TalonFX(Constants.Elevator.MOTOR_RIGHT_ID);
  private TalonFX motoLeft = new TalonFX(Constants.Elevator.MOTOR_LEFT_ID);

  private DigitalInput bottomLimitSwitch = new DigitalInput(Constants.Elevator.BOTTOM_LIMIT_SWITCH_ID);
  private DigitalInput topLimitSwitch = new DigitalInput(Constants.Elevator.TOP_LIMIT_SWITCH_ID);

  private double targetPosition;
  private double targetVelocity;
  private double finalTargetPosition;

  public Elevator() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    motorRight.getConfigurator().apply(config);
    motoLeft.getConfigurator().apply(config);
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.Elevator.PIDs.ELEVATOR_kP;
    slot0Configs.kI = Constants.Elevator.PIDs.ELEVATOR_kI;
    slot0Configs.kD = Constants.Elevator.PIDs.ELEVATOR_kD;
    slot0Configs.kV = Constants.Elevator.PIDs.ELEVATOR_kV;
    slot0Configs.kS = Constants.Elevator.PIDs.ELEVATOR_kS;
    motorRight.getConfigurator().apply(slot0Configs);
    motoLeft.getConfigurator().apply(slot0Configs);

    motorRight.setNeutralMode(NeutralModeValue.Brake);
    motoLeft.setNeutralMode(NeutralModeValue.Brake);

    targetPosition = getElevatorPosition();
    motoLeft.setControl(new Follower(Constants.Elevator.MOTOR_RIGHT_ID, true));
  }

  @Override
  public void periodic() {
    if(getBottomLimitSwitch()) {
      motorRight.setPosition(0.0);
    } else if(getTopLimitSwitch()) {
      motorRight.setPosition(Constants.Elevator.MAX_HEIGHT_CARRIAGE * Constants.Elevator.METERS_TO_MOTOR_ROTATIONS);
    }
    // putInfoSmartDashboard(); Used for Elevator info
  }

  // Reminder: all positions are measured in meters
  public double getElevatorPosition() {
    return motorRight.getPosition().getValueAsDouble() * Constants.Elevator.MOTOR_ROTATIONS_TO_METERS;
  }

  // Reminder: all velocities are measured in meters/second
  public double getElevatorVelocity() {
    return motorRight.getVelocity().getValueAsDouble() * Constants.Elevator.MOTOR_ROTATIONS_TO_METERS;
  }

  // Inverted because limit switches return true until tripped
  public boolean getBottomLimitSwitch() {
    return !bottomLimitSwitch.get();
  }

  // Inverted because limit switches return true until tripped
  public boolean getTopLimitSwitch() {
    return !topLimitSwitch.get();
  }

  public void setMotorTrapezoidProfileSafe(double position, double velocity) {
    PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    positionVoltage.Position = position;
    positionVoltage.Velocity = velocity;
    targetPosition = position;
    targetVelocity = velocity;

    motorRight.setControl(positionVoltage
      .withLimitForwardMotion(getTopLimitSwitch())
      .withLimitReverseMotion(getBottomLimitSwitch())
    );
  }

  public boolean isAtSetpoint() {
    return Math.abs(getElevatorPosition() - targetPosition) 
      < Constants.Elevator.TOLORENCE_METERS_FOR_SETPOINT;
  }

  public void setTargetPosition(double newTargetPosition) {
    targetPosition = newTargetPosition;
  }

  public void setFinalTargetPosition(double newFinalTargetPosition) {
    finalTargetPosition = newFinalTargetPosition;
  }

  public void putInfoSmartDashboard() {
    double currentPos = motorRight.getPosition().getValueAsDouble();

    SmartDashboard.putNumber("Elevator/Desired velocity in mps", targetVelocity);
    SmartDashboard.putNumber("Elevator/Desired position in meters", targetPosition);
    SmartDashboard.putNumber("Elevator/Desired position in rotations", targetPosition * Constants.Elevator.METERS_TO_MOTOR_ROTATIONS);
    SmartDashboard.putNumber("Elevator/Actual velocity in mps", motorRight.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Actual position in meters", currentPos * Constants.Elevator.MOTOR_ROTATIONS_TO_METERS);
    SmartDashboard.putNumber("Elevator/Actual position in rotations", currentPos);
    SmartDashboard.putNumber("Elevator/Final setpoint in meters", finalTargetPosition);
    SmartDashboard.putNumber("Elevator/Final setpoint in rotations", finalTargetPosition * Constants.Elevator.METERS_TO_MOTOR_ROTATIONS);
    SmartDashboard.putBoolean("Elevator/Bottom limit switch", getBottomLimitSwitch());
    SmartDashboard.putBoolean("Elevator/Top limit switch", getTopLimitSwitch());

  }
}
