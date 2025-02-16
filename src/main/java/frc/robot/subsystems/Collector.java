// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.utils.SmartPIDControllerTalonFX;

public class Collector extends SubsystemBase {

  private TalonFX rightMotor; 
  private TalonFX leftMotor; 

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private double lastRightSpeed;
  private double lastLeftSpeed;

  // Neither of these smart PIDs are 'used' after they are constructed because the
  // PID controller is built into the motor (we don't have to call .calculate like we
  // do with the PIDController class).
  @SuppressWarnings("unused")
  private SmartPIDControllerTalonFX rightMotorController;
  @SuppressWarnings("unused")
  private SmartPIDControllerTalonFX leftMotorController;

  private double getLeftMotorVelocity() {
    return leftMotor.getVelocity().getValueAsDouble();
  }
  private double getRightMotorVelocity() {
    return rightMotor.getVelocity().getValueAsDouble();
  }


  /** Creates a new Collector. */
  public Collector() {
   rightMotor = new TalonFX(Constants.Collector.RIGHT_MOTOR);
   leftMotor = new TalonFX(Constants.Collector.LEFT_MOTOR);
    
    TalonFXConfiguration talonConfigCollectorMotor = new TalonFXConfiguration();

    talonConfigCollectorMotor.MotorOutput.NeutralMode = NeutralModeValue.Brake;

   rightMotor.getConfigurator().apply(talonConfigCollectorMotor);
   leftMotor.getConfigurator().apply(talonConfigCollectorMotor);

   rightMotorController = new SmartPIDControllerTalonFX(Constants.Collector.PIDs.KP,
        Constants.Collector.PIDs.KI, Constants.Collector.PIDs.KD,
        Constants.Collector.PIDs.KF, "right motor",
        Constants.Collector.PIDs.SMART_PID_ENABLED, rightMotor);

    leftMotorController = new SmartPIDControllerTalonFX(Constants.Collector.PIDs.KP,
        Constants.Collector.PIDs.KI, Constants.Collector.PIDs.KD,
        Constants.Collector.PIDs.KF, "left motor",
        Constants.Drivebase.PIDs.SMART_PID_ENABLED, leftMotor);
    
  }

  // meters per sec 
  private void setCollectorSpeeds(double rightSpeed, double leftSpeed){
    if (rightSpeed != lastRightSpeed) {
      rightMotor.setControl(velocityVoltage
          .withVelocity(rightSpeed * Constants.Collector.COLLECTOR_ROTATIONS_PER_METER));
    SmartDashboard.putNumber("right speed", rightSpeed);
    }
    lastRightSpeed = rightSpeed;

    if (leftSpeed != lastLeftSpeed) {
      leftMotor.setControl(velocityVoltage
          .withVelocity(leftSpeed * Constants.Collector.COLLECTOR_ROTATIONS_PER_METER));
    SmartDashboard.putNumber("left speed", leftSpeed);
    }
    lastLeftSpeed = leftSpeed;
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Right motor current", rightMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Left motor current", leftMotor.getSupplyCurrent().getValueAsDouble());
  }
  
  public Command rotateCoralCommand() {
    return runEnd(
      () -> {
        setCollectorSpeeds(-Constants.Collector.COLLECOR_ROTATE_SLOW, 
        Constants.Collector.COLLECOR_ROTATE_FAST);
        SmartDashboard.putNumber("right collector current speed", getRightMotorVelocity());
        SmartDashboard.putNumber("left collector current speed", getLeftMotorVelocity());
      }, 
      () -> {
        setCollectorSpeeds(0, 0);
      }
    );
  }

  int endCount [] = {0};
  // true if you want it to stop the motor when the command ends
  // it should almost always be true unless there will be a following command right after that will end it
  public Command intakeCoralCommand(
    boolean stopOnEnd
  ) {
    return runEnd(
      () -> {
        setCollectorSpeeds(-Constants.Collector.COLLECOR_ROTATE_FAST, 
          Constants.Collector.COLLECOR_ROTATE_FAST* Constants.Collector.COLLECTOR_OFFSET);
      },
      () -> {
        if(stopOnEnd) {
          setCollectorSpeeds(0, 0);
        }
      }
    ).until(
      () -> {
        if (rightMotor.getSupplyCurrent().getValueAsDouble() >= Constants.Collector.COLLECTOR_AMPS_BEFORE_CUTTOF &&
        leftMotor.getSupplyCurrent().getValueAsDouble() >= Constants.Collector.COLLECTOR_AMPS_BEFORE_CUTTOF) 
        {
          endCount[0]++;
        }
        else
        {
          endCount[0] = 0;
        }
        return endCount[0] > 3;
      }
    );
  }

  public Command scorePieceCommand() {
    return runEnd(
      () -> {
        setCollectorSpeeds(-Constants.Collector.COLLECOR_ROTATE_FAST, 
          Constants.Collector.COLLECOR_ROTATE_FAST);
      },
      () -> {
        setCollectorSpeeds(0, 0);
      }
    );
  }

  public Command waitAfterCatchPieceCommand() {
    return Commands.sequence(
      intakeCoralCommand(false),
      Commands.race(
        scorePieceCommand(),
        Commands.waitSeconds(Constants.Collector.SECONDS_BEFORE_CUTTOF)
      )
    );
  }
}
