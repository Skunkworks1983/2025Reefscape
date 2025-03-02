// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CurrentLimits;
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

  /** Creates a new Collector. */
  public Collector() {
    rightMotor = new TalonFX(Constants.Collector.RIGHT_MOTOR);
    leftMotor = new TalonFX(Constants.Collector.LEFT_MOTOR);
    rightMotor.setPosition(0.0);
    leftMotor.setPosition(0.0);

    leftMotor.setInverted(true);
    setDefaultCommand(holdPositionCommand());

    TalonFXConfiguration talonConfigCollectorMotor = new TalonFXConfiguration();
    talonConfigCollectorMotor.CurrentLimits = CurrentLimits.KRAKEN_CURRENT_LIMIT_CONFIG;

    talonConfigCollectorMotor.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rightMotor.getConfigurator().apply(talonConfigCollectorMotor);
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

    leftMotorController.AddSlot1Configs(Constants.Collector.PositionControlMode.KP, //POSITION
        Constants.Collector.PositionControlMode.KI, Constants.Collector.PositionControlMode.KD,
        Constants.Collector.PositionControlMode.KF, Constants.Collector.PositionControlMode.KV,
        Constants.Collector.PositionControlMode.KA, Constants.Collector.PositionControlMode.KS);

    beambreak = new DigitalInput(Constants.Collector.DIGITAL_INPUT_CHANNEL);
  }

  // meters per sec
  private void setCollectorSpeeds(double rightSpeed, double leftSpeed) {
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
    ConditionalSmartDashboard.putBoolean("Collector/Beambreak collector", !beambreak.get());
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

  public void setCollectorSetPoint(double newRightSetpoint, double newLeftSetpoint) {
    rightMotor.setControl(
        positionVoltage.withPosition(newRightSetpoint).withSlot(1));
    leftMotor.setControl(
        positionVoltage.withPosition(newLeftSetpoint).withSlot(1));
  }

  public Command rotateCoralCommand() {
    return runEnd(
        () -> {
          setCollectorSpeeds(Constants.Collector.CORAL_INTAKE_SLOW_SPEED,
              Constants.Collector.CORAL_INTAKE_FAST_SPEED);
          ConditionalSmartDashboard.putNumber("Collector/right collector current speed", getRightMotorVelocity());
          ConditionalSmartDashboard.putNumber("Collector/left collector current speed", getLeftMotorVelocity());
        },
        () -> {
          setCollectorSpeeds(0, 0);
        });
  }

  int endCount[] = { 0 }; // This value needs to be effectivly final

  /*
    The intakeCoralCommand is intended to generally be used in a sequential command group 
    where it is immediately followed by the waitAfterCatchPieceCommand.
    When used in the sequential command group in that way, the stopOnEnd parameter should be false. 
    The stopOnEnd parameter should be true if the waitAfterCatchPiece command is not in a sequential 
    command group with intakeCoralCommand." 
   */
  public Command intakeCoralCommand(
      boolean stopOnEnd) {
    return runEnd(
        () -> {
          setCollectorSpeeds(Constants.Collector.CORAL_INTAKE_FAST_SPEED,
              Constants.Collector.CORAL_INTAKE_FAST_SPEED * Constants.Collector.SPEED_MULIPILER_LEFT);
        },
        () -> {
          if (stopOnEnd) {
            setCollectorSpeeds(0, 0);
          }
        }).until(
            () -> {
              ConditionalSmartDashboard.putNumber("Collector/Amp cut off right",
                  rightMotor.getSupplyCurrent().getValueAsDouble());
              ConditionalSmartDashboard.putNumber("Collector/Amp cut off left",
                  leftMotor.getSupplyCurrent().getValueAsDouble());
              if (rightMotor.getSupplyCurrent().getValueAsDouble() >= Constants.Collector.COLLECTOR_AMPS_BEFORE_CUTTOF
                  ||
                  leftMotor.getSupplyCurrent().getValueAsDouble() >= Constants.Collector.COLLECTOR_AMPS_BEFORE_CUTTOF ||
                  rightMotor.getSupplyCurrent().getValueAsDouble() < 0 ||
                  leftMotor.getSupplyCurrent().getValueAsDouble() < 0) {
                endCount[0]++;
              } else {
                endCount[0] = 0;
              }
              return endCount[0] >= Constants.Collector.END_COUNT_TICK_COUNTER;
            });
  }

  public Command scoreCoralCommand() {
    return runEnd(
        () -> {
          setCollectorSpeeds(Constants.Collector.CORAL_INTAKE_FAST_SPEED,
              Constants.Collector.CORAL_INTAKE_FAST_SPEED);
        },
        () -> {
          setCollectorSpeeds(0, 0);
        }).until(
            () -> {
              if (rightMotor.getSupplyCurrent().getValueAsDouble() >= Constants.Collector.COLLECTOR_AMPS_BEFORE_CUTTOF
                  &&
                  leftMotor.getSupplyCurrent().getValueAsDouble() >= Constants.Collector.COLLECTOR_AMPS_BEFORE_CUTTOF) {
                endCount[0]++;
              } else {
                endCount[0] = 0;
              }
              return endCount[0] > Constants.Collector.END_COUNT_TICK_COUNTER;
            });
  }

  public Command waitAfterCatchPieceCommand() {
    return Commands.sequence(
        intakeCoralCommand(false),
        Commands.race(
            scoreCoralCommand(),
            Commands.waitSeconds(Constants.Collector.SECONDS_BEFORE_CUTTOF)));
  }

  public Command expelCoral(boolean stopOnEnd) {
    return runEnd(
        () -> {
          setCollectorSpeeds(Constants.Collector.COLLECTOR_REVERSE,
              Constants.Collector.COLLECTOR_REVERSE);
        },
        () -> {
          if (stopOnEnd) {
            setCollectorSpeeds(0, 0);
          }
        });

  }

  public Command holdPositionCommand() {
    return startEnd(
        () -> {
          setCollectorSetPoint(getRightMotorPosition(), getLeftMotorPosition());
        },
        () -> {

        });
  }

  public Command intakeAlgaeCommand(
      boolean stopOnEnd) {
    return runEnd(
        () -> {
          setCollectorSpeeds(Constants.Collector.ALGAE_INTAKE,
              Constants.Collector.ALGAE_INTAKE);
        },
        () -> {
          if (stopOnEnd) {
            setCollectorSpeeds(0, 0);
          }
        }).until(
            () -> {
              if (rightMotor.getSupplyCurrent().getValueAsDouble() >= Constants.Collector.ALGAE_AMP_CUT_OFF &&
                  leftMotor.getSupplyCurrent().getValueAsDouble() >= Constants.Collector.ALGAE_AMP_CUT_OFF) {
                endCount[0]++;
              } else {
                endCount[0] = 0;
              }
              return endCount[0] > Constants.Collector.END_COUNT_TICK_COUNTER;
            });
  }

  public Command expelAlgaeCommand(
      boolean stopOnEnd) {
    return runEnd(
        () -> {
          setCollectorSpeeds(Constants.Collector.ALGAE_EXPEL,
              Constants.Collector.ALGAE_EXPEL);
        },
        () -> {
          if (stopOnEnd) {
            setCollectorSpeeds(0, 0);
          }
        });
  }

}