// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.errorCommands.TestModuleComponentsConnection;
import frc.robot.Commands.errorCommands.TestTurnMotorAndEncoder;
import frc.robot.constants.Constants;
import frc.robot.constants.SwerveModuleConstants;
import frc.robot.utils.SmartPIDController;
import frc.robot.utils.SmartPIDControllerTalonFX;
import frc.robot.utils.error.ErrorGroupHandler;

public class SwerveModule extends SubsystemBase {

  private SparkMax turnMotor;
  private TalonFX driveMotor;
  private CANcoder turnEncoder;
  private SmartPIDControllerTalonFX driveController;
  private SmartPIDController turnController;
  private boolean turnControllerActive;

  public String moduleName;
  public Translation2d moduleLocation;

  final VelocityVoltage m_Velocity = new VelocityVoltage(0);

  public SwerveModule(SwerveModuleConstants swerveConstants) {
    this(
      swerveConstants.driveMotorId, 
      swerveConstants.turnMotorId, 
      swerveConstants.turnEncoderId, 
      swerveConstants.turnEncoderOffset, 
      swerveConstants.moduleLocation, 
      swerveConstants.moduleName
    );
  }
  
  public SwerveModule(
    int driveModuleId, 
    int turnModuleId, 
    int turnEncoderId,
    double turnEncoderOffset, 
    Translation2d moduleLocation, 
    String moduleName
  ) {
    this.driveMotor = new TalonFX(driveModuleId, Constants.Drivebase.CANIVORE_NAME);
    this.turnMotor = new SparkMax(turnModuleId, MotorType.kBrushless);
    this.turnEncoder = new CANcoder(turnEncoderId, Constants.Drivebase.CANIVORE_NAME);
    this.moduleLocation = moduleLocation;
    this.moduleName = moduleName;

    turnController = new SmartPIDController(
      Constants.Drivebase.PIDs.SWERVE_MODULE_TURN_kP,
      Constants.Drivebase.PIDs.SWERVE_MODULE_TURN_kI, 
      Constants.Drivebase.PIDs.SWERVE_MODULE_TURN_kD, 
      moduleName + " Turn", 
      Constants.Drivebase.PIDs.SMART_PID_TURN_ENABLED
    );
    turnController.enableContinuousInput(-180, 180);
    turnController.setTolerance(0.005);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveMotor.getConfigurator().apply(driveConfig);

    driveController = new SmartPIDControllerTalonFX(
      Constants.Drivebase.PIDs.SWERVE_MODULE_DRIVE_kP,
      Constants.Drivebase.PIDs.SWERVE_MODULE_DRIVE_kI, 
      Constants.Drivebase.PIDs.SWERVE_MODULE_DRIVE_kD,
      Constants.Drivebase.PIDs.SWERVE_MODULE_DRIVE_kF, 
      moduleName + " Drive",
      Constants.Drivebase.PIDs.SMART_PID_DRIVE_ENABLED, 
      driveMotor
    );

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(true);
    turnMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    CANcoderConfiguration encoder = new CANcoderConfiguration();
    encoder.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    encoder.MagnetSensor.MagnetOffset = -turnEncoderOffset;
    turnEncoder.getConfigurator().apply(encoder);

    m_Velocity.Slot = 0;
  }

  @Override
  public void periodic() {
    driveController.updatePID();
    if (!turnController.atSetpoint() && turnControllerActive) {
      updateSpeedToSetpointTurn();
    }
  }

  public void setTurnControllerActive(boolean isControllerActive) {
    turnControllerActive = isControllerActive;
  }

  public void setModuleTurnSetpoint(Rotation2d angle) {
    turnController.setSetpoint(angle.getDegrees());
  }

  public void setModuleDriveVelocity(double metersPerSecond) {
    driveMotor.setControl(m_Velocity.withVelocity(metersPerSecond * Constants.Drivebase.Info.REVS_PER_METER)
      .withEnableFOC(true));
  }

  // returns meters traveled
  public double getDriveMotorEncoderPosition() {
    return driveMotor.getPosition().getValueAsDouble() / Constants.Drivebase.Info.REVS_PER_METER;
  }

  public double getTurnMotorEncoderPosition() {
    return turnMotor.getEncoder().getPosition() / Constants.Drivebase.Info.TURN_MOTOR_GEAR_RATIO; 
  }

  // returns velocity in meters
  public double getDriveMotorVelocity() {
    return driveMotor.getVelocity().getValueAsDouble() / Constants.Drivebase.Info.REVS_PER_METER;
  }

  public void setTurnMotorSpeed(double speed) {
    turnMotor.set(speed);
  }

  public void setBrakeMode(boolean brakeMode) {
    if(brakeMode) {
      driveMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    else {
      driveMotor.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  // Called in periodic if not at setpoint to recalculate speed
  public void updateSpeedToSetpointTurn() {
    setTurnMotorSpeed(MathUtil.clamp(turnController.calculate(getTurnMotorAngle().getDegrees()),
      Constants.Drivebase.PIDs.PID_LOW_LIMIT,
      Constants.Drivebase.PIDs.PID_HIGH_LIMIT));
  }

  // Returns turn encoders absolute position between 180 to -180 degrees
  public Rotation2d getTurnMotorAngle() {
    Rotation2d turnMotorRotation = Rotation2d.fromRotations(turnEncoder.getAbsolutePosition().getValueAsDouble());
    return turnMotorRotation;
  }

  public double getTurnMotorCurrent() {
    return turnMotor.getBusVoltage();
  }

  public boolean isEncoderConnected() {
    return turnEncoder.isConnected();
  }

  public double getTurnError() {
    return turnController.getPositionError();
  }

  public double getRawEncoderValue() {
    return turnEncoder.getPosition().getValueAsDouble();
  }

  public void setSwerveModulState(SwerveModuleState newState) {
    // Makes sure we are turning the lowest ammount to get to the desired angle
    SwerveModuleState newStateOptimized = newState;
    newStateOptimized.optimize(getTurnMotorAngle());

    // Modifies our desired drive motor velocity by how far off the angle is
    newStateOptimized.speedMetersPerSecond *= newStateOptimized.angle.minus(getTurnMotorAngle()).getCos();
    setModuleDriveVelocity(newStateOptimized.speedMetersPerSecond);

    // sets the desired turn motor angle to the new angle
    Rotation2d turnMotorAngleOptimized = new Rotation2d(newStateOptimized.angle.getRadians());
    setModuleTurnSetpoint(turnMotorAngleOptimized);
  }

  public SwerveModuleState getSwerveModuleState() {
    return new SwerveModuleState(getDriveMotorVelocity(), getTurnMotorAngle());
  }

  public SwerveModulePosition getSwerveModulePosition() {
    return new SwerveModulePosition(getDriveMotorEncoderPosition(),
      getTurnMotorAngle());
  }

  @Override
  public String toString() {
    return moduleName + " Module";
  }

  public boolean isTurnMotorConnected() {
    return turnMotor.getFirmwareVersion() != 0;
  }

  public boolean isDriveMotorConnected() {
    return driveMotor.isConnected();
  }

  public Command TestConnectionThenModule(
    ErrorGroupHandler errorGroupHandler
  ) {
    return Commands.sequence(
      new TestModuleComponentsConnection(errorGroupHandler::addErrorMapEntry, this),
      Commands.either(
        Commands.race(
          new TestTurnMotorAndEncoder(errorGroupHandler::addErrorMapEntry, this),
          Commands.waitSeconds(5)
        ),
        Commands.none(),
        () -> !errorGroupHandler.getErrorStatus("Turn Encoder Not Connected", this) && 
              !errorGroupHandler.getErrorStatus("Turn Motor Not Connected", this)
      )
    );
  }
}