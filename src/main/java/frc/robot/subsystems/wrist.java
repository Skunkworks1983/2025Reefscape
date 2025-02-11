// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.utils.SmartPIDControllerTalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Wrist extends SubsystemBase {
  TalonFX wristMotor;
  double degrees;
  double pidSpeed;
  double degreesSetpoint;

  private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
  // TODO figure out if position 0 is starting position

  private double targetDegrees = getWristPosition();
  @SuppressWarnings("unused")
  private SmartPIDControllerTalonFX wristMotorController;

  /** Creates a new wrist. */
  public Wrist(int wristMotorID, int canCoderId) {
    wristMotor = new TalonFX(wristMotorID);
    // wristMotor.setNeutralMode(NeutralModeValue.Brake);
    wristMotorController = new SmartPIDControllerTalonFX(Constants.Wrist.PIDs.WRIST_MOTOR_kP,
        Constants.Wrist.PIDs.WRIST_MOTOR_kI, Constants.Wrist.PIDs.WRIST_MOTOR_kD, Constants.Wrist.PIDs.WRIST_MOTOR_kF,
        "wrist motor", Constants.Wrist.PIDs.SMART_PID_ENABLED, wristMotor);
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

  private boolean isAtSetpoint() {
    return Math.abs(getWristPosition() - targetDegrees) < Constants.Wrist.TOLERENCE_DEGREES_FOR_SETPOINT;
  }

  // public Command retainTargetPosition()
  // {
  //   return run(
  //     () -> {
  //         double velocity = getWristVelocity()
  //         .getVelocity(
  //             getWristPosition(), 
  //             targetDegrees
  //         );
  //     }
  //   );
  // }
}
