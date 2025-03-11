// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CurrentLimits;
import frc.robot.constants.EndEffectorSetpointConstants;
import frc.robot.utils.ConditionalSmartDashboard;
import frc.robot.utils.PIDControllers.SmartPIDControllerTalonFX;

public class Collector extends SubsystemBase {

  private TalonFX rightMotor;
  private TalonFX leftMotor;

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private double lastRightSpeed;
  private double lastLeftSpeed;

  // Neither of these smart PIDs are 'used' after they are constructed because the
  // PID controller is built into the motor (we don't have to call .calculate like
  // we
  // do with the PIDController class).
  @SuppressWarnings("unused")
  private SmartPIDControllerTalonFX rightMotorController;
  @SuppressWarnings("unused")
  private SmartPIDControllerTalonFX leftMotorController;

  private DigitalInput beambreak;

  double collectorRightSetpoint;
  double collectorLeftSetPoint;
  private PositionVoltage positionVoltage = new PositionVoltage(0);
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
  double lastThrottle = 0.0;

  /** Creates a new Collector. */
  public Collector() {
   rightMotor = new TalonFX(Constants.Collector.IDs.RIGHT_MOTOR, "Collector 2025");
   leftMotor = new TalonFX(Constants.Collector.IDs.LEFT_MOTOR, "Collector 2025");

    setDefaultCommand(holdPositionCommand());

    TalonFXConfiguration talonConfigCollectorMotor = new TalonFXConfiguration();
    talonConfigCollectorMotor.CurrentLimits = CurrentLimits.KRAKEN_CURRENT_LIMIT_CONFIG;

    talonConfigCollectorMotor.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightMotor.getConfigurator().apply(talonConfigCollectorMotor);
    talonConfigCollectorMotor.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftMotor.getConfigurator().apply(talonConfigCollectorMotor);

    //Dual Pids with two slots, one for velocity and one for position
    rightMotorController = new SmartPIDControllerTalonFX(Constants.Collector.VelocityControlMode.KP, //VELOCITY
        Constants.Collector.VelocityControlMode.KI, Constants.Collector.VelocityControlMode.KD,
        Constants.Collector.VelocityControlMode.KF, Constants.Collector.VelocityControlMode.KV,
        Constants.Collector.VelocityControlMode.KA, Constants.Collector.VelocityControlMode.KS,
        "right motor",
        Constants.Collector.SMART_PID_ENABLED, rightMotor);
    rightMotorController.AddSlot1Configs(Constants.Collector.PositionControlMode.KP, //POSITION
        Constants.Collector.PositionControlMode.KI, Constants.Collector.PositionControlMode.KD,
        Constants.Collector.PositionControlMode.KF, Constants.Collector.PositionControlMode.KV,
        Constants.Collector.PositionControlMode.KA, Constants.Collector.PositionControlMode.KS);

    leftMotorController = new SmartPIDControllerTalonFX(Constants.Collector.VelocityControlMode.KP, //VELOCITY
        Constants.Collector.VelocityControlMode.KI,
        Constants.Collector.VelocityControlMode.KD,
        Constants.Collector.VelocityControlMode.KF, Constants.Collector.VelocityControlMode.KV,
        Constants.Collector.VelocityControlMode.KA, Constants.Collector.VelocityControlMode.KS,
        "left motor",
        Constants.Collector.SMART_PID_ENABLED, leftMotor);
    
        beambreak = new DigitalInput(Constants.Collector.IDs.DIGITAL_INPUT_CHANNEL);
  }

  private void setCollectorThrottle(double throttle) {

    if(throttle != lastThrottle) {
      leftMotor.setControl(dutyCycleOut.withOutput(throttle).withEnableFOC(true));
      rightMotor.setControl(dutyCycleOut.withOutput(throttle).withEnableFOC(true));
      lastThrottle = throttle;
    }
  }

  // meters per sec
  private void setCollectorSpeeds(double rightSpeed, double leftSpeed) {
    // Reseting last throttle
    lastThrottle = 0;
    if (rightSpeed != lastRightSpeed) {
      rightMotor.setControl(velocityVoltage
          .withVelocity(rightSpeed * Constants.Collector.COLLECTOR_ROTATIONS_PER_METER).withEnableFOC(true)
          .withSlot(0));
      ConditionalSmartDashboard.putNumber("Collector/Right speed", rightSpeed);
    }
    lastRightSpeed = rightSpeed;

    if (leftSpeed != lastLeftSpeed) {
      leftMotor.setControl(velocityVoltage
          .withVelocity(leftSpeed * Constants.Collector.COLLECTOR_ROTATIONS_PER_METER).withEnableFOC(true).withSlot(0));
      ConditionalSmartDashboard.putNumber("Collector/Left speed", leftSpeed);
    }
    lastLeftSpeed = leftSpeed;
  }

  @Override
  public void periodic() {
    leftMotorController.updatePID();
    rightMotorController.updatePID();

    ConditionalSmartDashboard.putNumber("Collector/Right motor current",
        rightMotor.getSupplyCurrent().getValueAsDouble());
    ConditionalSmartDashboard.putNumber("Collector/Left motor current",
        leftMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putBoolean("Collector/Beambreak collector", !beambreak.get());
  }

  public double getLeftMotorVelocity() {
    return leftMotor.getVelocity().getValueAsDouble();
  }

  public double getRightMotorVelocity() {
    return rightMotor.getVelocity().getValueAsDouble();
  }

  public double getRightMotorPosition() {
    return rightMotor.getPosition().getValueAsDouble();
  }

  public double getLeftMotorPosition() {
    return leftMotor.getPosition().getValueAsDouble();
  }

  public boolean isHoldingCoral() {
    return !beambreak.get();
  }

  public void setCollectorSetPoint(double newRightSetpoint, double newLeftSetpoint) {
    rightMotor.setControl(
        positionVoltage.withPosition(newRightSetpoint).withSlot(1));
    leftMotor.setControl(
        positionVoltage.withPosition(newLeftSetpoint).withSlot(1));
  }

  public Command rotateCoralCommand() {
    return runEnd(
      () -> {
        setCollectorSpeeds(Constants.Collector.Speeds.CORAL_INTAKE_SLOW_SPEED, 
        Constants.Collector.Speeds.CORAL_INTAKE_FAST_SPEED);
        ConditionalSmartDashboard.putNumber("Collector/Right collector current speed", getRightMotorVelocity());
        ConditionalSmartDashboard.putNumber("Collector/Left collector current speed", getLeftMotorVelocity());
      }, 
      () -> {
        setCollectorSpeeds(0, 0);
      }
    );
  }


  // Rrue if you want it to stop the motor when the command ends
  // it should almost always be true unless there will be a following command right after that will end it
  public Command intakeCoralCommand(
    boolean stopOnEnd
    ) {
    int endCount [] = {0}; // This value needs to be effectivly final 
    return runEnd(
      () -> {
        setCollectorSpeeds(Constants.Collector.Speeds.CORAL_INTAKE_SLOW_SPEED, 
          Constants.Collector.Speeds.CORAL_INTAKE_SLOW_SPEED);
      },
      () -> {
        if(stopOnEnd) {
          setCollectorSpeeds(0, 0);
        }
      }
    ).beforeStarting(
      () -> {
        endCount[0] = 0;
      }
    ).until(
      () -> {
        ConditionalSmartDashboard.putNumber("Collector/Amp cut off right", rightMotor.getSupplyCurrent().getValueAsDouble());
        ConditionalSmartDashboard.putNumber("Collector/Amp cut off left", leftMotor.getSupplyCurrent().getValueAsDouble());
        if (!beambreak.get()) 
        {
          endCount[0]++;
        }
        else
        {
          endCount[0] = 0;
        }
        return endCount[0] >= Constants.Collector.END_COUNT_TICK_COUNTER_CORAL;
      }
    );
  }

  public Command expelCoralCommand(
    boolean stopOnEnd,
    Supplier<EndEffectorSetpointConstants> endEffectorSetpoint
    )
  {
    return runEnd(
      () -> {
          setCollectorSpeeds(Constants.Collector.Speeds.CORAL_EXPEL_FAST_SPEED, 
            Constants.Collector.Speeds.CORAL_EXPEL_FAST_SPEED);
      },
      () -> {
        if(stopOnEnd) {
          setCollectorSpeeds(0, 0);
        }
      }
    );

  }

  public Command holdPositionCommand(){
    return startEnd(
      () -> {
        setCollectorSetPoint(getRightMotorPosition(), getLeftMotorPosition());
      }, () -> {

      }
    );
  }

  public Command intakeAlgaeCommand(
    boolean stopOnEnd
  ) {
    int endCount [] = {0}; // This value needs to be effectivly final 
    return runEnd(
      () -> {
        // setCollectorSpeeds(Constants.Collector.Speeds.ALGAE_INTAKE_SPEED, 
        //   Constants.Collector.Speeds.ALGAE_INTAKE_SPEED);
        setCollectorThrottle(Constants.Collector.Speeds.ALGAE_INTAKE_SPEED);
      },
      () -> {
        if(stopOnEnd) {
          setCollectorSpeeds(0, 0);
        }
      }
    ).beforeStarting(
      () -> {
        endCount[0] = 0;
      }
    ).until(
      () -> {
        if (rightMotor.getSupplyCurrent().getValueAsDouble() >= Constants.Collector.ALGAE_AMP_CUT_OFF &&
        leftMotor.getSupplyCurrent().getValueAsDouble() >= Constants.Collector.ALGAE_AMP_CUT_OFF) 
        {
          endCount[0]++;
        }
        else
        {
          endCount[0] = 0;
        }
        return endCount[0] > Constants.Collector.END_COUNT_TICK_COUNTER_ALGAE;
      }
    );
  }

  public Command expelAlgaeCommand(
      boolean stopOnEnd) {
    return runEnd(
      () -> {
        setCollectorSpeeds(Constants.Collector.Speeds.ALGAE_EXPEL_SPEED, 
          Constants.Collector.Speeds.ALGAE_EXPEL_SPEED);
      },
      () -> {
        if(stopOnEnd) {
          setCollectorSpeeds(0, 0);
        }
      }
    );
  }

}