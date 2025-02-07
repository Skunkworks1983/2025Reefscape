// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Wrist.WristProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import java.io.ObjectInputFilter.Config;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
// import com.studica.frc.AHRS;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class wrist extends SubsystemBase {
  TalonFX wristMotor;
  CANcoder canCoder;
  double degrees;
  PIDController pidWrist;
  double pidSpeed;
  double degreesSetpoint;

  private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
  // TODO figure out if position 0 is starting position

  private double targetDegrees = getWristPosition();
  final TrapezoidProfile motionProfile = new TrapezoidProfile(
      new Constraints(WristProfile.MAX_VELOCITY, WristProfile.MAX_ACCELERATION));

  /** Creates a new wrist. */
  public wrist(int wristMotorID, int canCoderId) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    wristMotor = new TalonFX(wristMotorID);
    // wristMotor.setNeutralMode(NeutralModeValue.Brake);
    pidWrist = new PIDController(
    Constants.Wrist.PIDs.WRIST_MOTOR_kP,
    Constants.Wrist.PIDs.WRIST_MOTOR_kI,
    Constants.Wrist.PIDs.WRIST_MOTOR_kD,
    Constants.Wrist.PIDs.WRIST_MOTOR_kF
    );
    canCoder = new CANcoder(canCoderId, "Practice Swerve");
  }

  public void setWristAnglePosition(double degrees) {
    double pos = degrees / Constants.Wrist.WRIST_REVS_TO_DEGREES;
    degreesSetpoint = pos;
    System.out.println("setting target wrist: " + pos);
    wristMotor.setControl(positionVoltage.withPosition(pos));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private double getWristPosition() {
    return wristMotor.getPosition().getValueAsDouble() * Constants.Wrist.WRIST_REVS_TO_DEGREES;
  }
  private double getWristVelocity() {
    return wristMotor.getVelocity().getValueAsDouble() * Constants.Wrist.WRIST_REVS_TO_DEGREES;
  }
  private boolean isAtSetpoint(){
    return Math.abs(getWristPosition() - targetDegrees) 
      < Constants.Wrist.TOLORENCE_DEGREES_FOR_SETPOINT;
  }
}
