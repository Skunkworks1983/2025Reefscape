// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
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

  private DigitalInput beambreak;

  /** Creates a new Collector. */
  public Collector() {
   rightMotor = new TalonFX(Constants.Collector.IDs.RIGHT_MOTOR);
   leftMotor = new TalonFX(Constants.Collector.IDs.LEFT_MOTOR);

   leftMotor.setInverted(true);
   
    
    TalonFXConfiguration talonConfigCollectorMotor = new TalonFXConfiguration();
    talonConfigCollectorMotor.CurrentLimits = CurrentLimits.KRAKEN_CURRENT_LIMIT_CONFIG;


    talonConfigCollectorMotor.MotorOutput.NeutralMode = NeutralModeValue.Brake;

   rightMotor.getConfigurator().apply(talonConfigCollectorMotor);
   leftMotor.getConfigurator().apply(talonConfigCollectorMotor);

   rightMotorController = new SmartPIDControllerTalonFX(Constants.Collector.PIDs.KP,
        Constants.Collector.PIDs.KI, Constants.Collector.PIDs.KD,
        Constants.Collector.PIDs.KF, Constants.Collector.PIDs.KV,
        Constants.Collector.PIDs.KA, Constants.Collector.PIDs.KS,
         "right motor",
        Constants.Collector.PIDs.SMART_PID_ENABLED, rightMotor);

    leftMotorController = new SmartPIDControllerTalonFX(Constants.Collector.PIDs.KP,
        Constants.Collector.PIDs.KI, Constants.Collector.PIDs.KD,
        Constants.Collector.PIDs.KF, Constants.Collector.PIDs.KV,
        Constants.Collector.PIDs.KA, Constants.Collector.PIDs.KS,
        "left motor",
        Constants.Collector.PIDs.SMART_PID_ENABLED, leftMotor);
    
        beambreak = new DigitalInput(Constants.Collector.IDs.DIGITAL_INPUT_CHANNEL);
  }

  // meters per sec 
  private void setCollectorSpeeds(double rightSpeed, double leftSpeed){
    if (rightSpeed != lastRightSpeed) {
      rightMotor.setControl(velocityVoltage
          .withVelocity(rightSpeed * Constants.Collector.COLLECTOR_ROTATIONS_PER_METER).withEnableFOC(true));
      ConditionalSmartDashboard.putNumber("Collector/ Right speed", rightSpeed);
    }
    lastRightSpeed = rightSpeed;

    if (leftSpeed != lastLeftSpeed) {
      leftMotor.setControl(velocityVoltage
          .withVelocity(leftSpeed * Constants.Collector.COLLECTOR_ROTATIONS_PER_METER).withEnableFOC(true));
      ConditionalSmartDashboard.putNumber("Collector/ Left speed", leftSpeed);
    }
    lastLeftSpeed = leftSpeed;
  }
  @Override
  public void periodic() {
    leftMotorController.updatePID();
    rightMotorController.updatePID();
    
    ConditionalSmartDashboard.putNumber("Collector/ Right motor current", rightMotor.getSupplyCurrent().getValueAsDouble());
    ConditionalSmartDashboard.putNumber("Collector/ Left motor current", leftMotor.getSupplyCurrent().getValueAsDouble());
    ConditionalSmartDashboard.putBoolean("Collector/ Beambreak collector", !beambreak.get());
  }
  
  public Command rotateCoralCommand() {
    return runEnd(
      () -> {
        setCollectorSpeeds(Constants.Collector.Speeds.CORAL_INTAKE_SLOW_SPEED, 
        Constants.Collector.Speeds.CORAL_INTAKE_FAST_SPEED);
        ConditionalSmartDashboard.putNumber("Collector/ right collector current speed", getRightMotorVelocity());
        ConditionalSmartDashboard.putNumber("Collector/ left collector current speed", getLeftMotorVelocity());
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
        setCollectorSpeeds(Constants.Collector.Speeds.CORAL_INTAKE_FAST_SPEED, 
          Constants.Collector.Speeds.CORAL_INTAKE_FAST_SPEED * Constants.Collector.Speeds.SPEED_MULIPILER_LEFT);
      },
      () -> {
        if(stopOnEnd) {
          setCollectorSpeeds(0, 0);
        }
      }
    ).until(
      () -> {
        ConditionalSmartDashboard.putNumber("Collector/ amp cut off right", rightMotor.getSupplyCurrent().getValueAsDouble());
        ConditionalSmartDashboard.putNumber("Collector/ amp cut off left", leftMotor.getSupplyCurrent().getValueAsDouble());
        if (rightMotor.getSupplyCurrent().getValueAsDouble() >= Constants.Collector.COLLECTOR_AMPS_BEFORE_CUTTOF ||
            leftMotor.getSupplyCurrent().getValueAsDouble() >= Constants.Collector.COLLECTOR_AMPS_BEFORE_CUTTOF ||
            rightMotor.getSupplyCurrent().getValueAsDouble() < 0 ||
            leftMotor.getSupplyCurrent().getValueAsDouble() < 0) 
        {
          endCount[0]++;
        }
        else
        {
          endCount[0] = 0;
        }
        return endCount[0] >= Constants.Collector.END_COUNT_TICK_COUNTER;
      }
    );
  }

  public Command expelCoralCoral(boolean stopOnEnd)
  {
    return runEnd(
      () -> {
        setCollectorSpeeds(Constants.Collector.Speeds.CORAL_EXPEL_SLOW_SPEED, 
          Constants.Collector.Speeds.CORAL_EXPEL_SLOW_SPEED);
      },
      () -> {
        if(stopOnEnd) {
          setCollectorSpeeds(0, 0);
        }
      }
    );

  }
  public Command intakeAlgaeCommand(
    boolean stopOnEnd
  ) {
    int endCount [] = {0}; // This value needs to be effectivly final 
    return runEnd(
      () -> {
        setCollectorSpeeds(Constants.Collector.Speeds.ALGAE_INTAKE_SPEED, 
          Constants.Collector.Speeds.ALGAE_INTAKE_SPEED);
      },
      () -> {
        if(stopOnEnd) {
          setCollectorSpeeds(0, 0);
        }
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
        return endCount[0] > Constants.Collector.END_COUNT_TICK_COUNTER;
      }
    );
  }

  public Command expelAlgaeCommand(
    boolean stopOnEnd
  ) {
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