// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutomatedTests.TestModuleComponentsConnection;
import frc.robot.commands.AutomatedTests.TestTurnMotorAndEncoderOnModule;
import frc.robot.constants.Constants;
import frc.robot.constants.SwerveModuleConstants;
import frc.robot.utils.PIDControllers.SmartPIDController;
import frc.robot.utils.PIDControllers.SmartPIDControllerTalonFX;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.Phoenix6Odometry;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState.Phoenix6SwerveModuleState;
import frc.robot.utils.error.ErrorGroup;

public class SwerveModule extends SubsystemBase {

  private TalonFX turnMotor;
  private TalonFX driveMotor;
  private CANcoder turnEncoder;
  private SmartPIDControllerTalonFX driveController;
  private SmartPIDController turnController;
  private boolean turnControllerActive;

  public String moduleName;
  public Translation2d moduleLocation;

  final VelocityVoltage m_Velocity = new VelocityVoltage(0);

  final private Phoenix6SwerveModuleState state;

  public SwerveModule(
    SwerveModuleConstants swerveConstants,
    Phoenix6Odometry phoenix6Odometry
  ) {
    this(
      swerveConstants.driveMotorId, 
      swerveConstants.turnMotorId, 
      swerveConstants.turnEncoderId, 
      swerveConstants.turnEncoderOffset, 
      swerveConstants.moduleLocation, 
      swerveConstants.moduleName,
      phoenix6Odometry
    );
  }
  
  public SwerveModule(
    int driveModuleId, 
    int turnModuleId, 
    int turnEncoderId,
    double turnEncoderOffset, 
    Translation2d moduleLocation, 
    String moduleName,
    Phoenix6Odometry phoenix6Odometry
  ) {
    this.driveMotor = new TalonFX(driveModuleId, Constants.Drivebase.CANIVORE_NAME);
    this.turnMotor = new TalonFX(turnModuleId, Constants.Drivebase.CANIVORE_NAME);
    this.turnEncoder = new CANcoder(turnEncoderId, Constants.Drivebase.CANIVORE_NAME);
    this.moduleLocation = moduleLocation;
    this.moduleName = moduleName;

    turnController = new SmartPIDController(
      Constants.Drivebase.PIDs.SWERVE_MODULE_TURN_KP,
      Constants.Drivebase.PIDs.SWERVE_MODULE_TURN_KI, 
      Constants.Drivebase.PIDs.SWERVE_MODULE_TURN_KD, 
      moduleName + " Turn", 
      Constants.Drivebase.PIDs.SMART_PID_TURN_ENABLED
    );
    turnController.enableContinuousInput(-180, 180);
    turnController.setTolerance(0.0);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveMotor.getConfigurator().apply(driveConfig);

    driveController = new SmartPIDControllerTalonFX(
      Constants.Drivebase.PIDs.SWERVE_MODULE_DRIVE_KP,
      Constants.Drivebase.PIDs.SWERVE_MODULE_DRIVE_KI, 
      Constants.Drivebase.PIDs.SWERVE_MODULE_DRIVE_KD,
      Constants.Drivebase.PIDs.SWERVE_MODULE_DRIVE_KF,
      Constants.Drivebase.PIDs.SWERVE_MODULE_DRIVE_KV,
      Constants.Drivebase.PIDs.SWERVE_MODULE_DRIVE_KA,
      Constants.Drivebase.PIDs.SWERVE_MODULE_DRIVE_KS,
      moduleName + " Drive",
      Constants.Drivebase.PIDs.SMART_PID_DRIVE_ENABLED, 
      driveMotor
    );

    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turnMotor.getConfigurator().apply(turnConfig);

    CANcoderConfiguration encoder = new CANcoderConfiguration();
    encoder.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    encoder.MagnetSensor.MagnetOffset = -turnEncoderOffset;
    turnEncoder.getConfigurator().apply(encoder);

    m_Velocity.Slot = 0;

    state = phoenix6Odometry.registerSwerveModule(
      turnEncoder,
      driveMotor
    );
  }

  @Override
  public void periodic() {
    driveController.updatePID();
    if (turnControllerActive && !turnController.atSetpoint()) {
      updateSpeedToSetpointTurn();
    }

    SmartDashboard.putNumber("Drivebase/Swerve Module orientation",getTurnMotorAngle().getDegrees());
  }

  // only used if we want to run a manual speed on the motor
  // the turn controller would overwrite it if we dont turn it off first
  public void setTurnControllerActive(boolean isControllerActive) {
    turnControllerActive = isControllerActive;
  }

  public void setModuleTurnSetpoint(Rotation2d angle) {
    turnController.setSetpoint(angle.getDegrees());
  }

  public void setModuleDriveVelocity(double metersPerSecond) {
    driveMotor.setControl(
      m_Velocity.withVelocity(
        metersPerSecond * Constants.Drivebase.Info.REVS_PER_METER
      ).withEnableFOC(true)
    );
  }

  // Almost nothing should be calling this exept tests, this gets position from the turn motor
  // what should be used would be getTurnMotorAngle()
  public double getTurnMotorEncoderPosition() {
    if(!DriverStation.isTestEnabled()) {
      new Alert("getTurnMotorEncoderPosition was called outside of Test", AlertType.kError).set(true);
      return getTurnMotorAngle().getDegrees();
    }
    return turnMotor.getPosition().getValueAsDouble() / Constants.Drivebase.Info.TURN_MOTOR_GEAR_RATIO; 
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

  // Returns voltage in volts
  public double getTurnMotorVoltage() {
    return turnMotor.getMotorVoltage().getValueAsDouble();
  }

  // Returns Amps
  public double getTurnMotorCurrent() {
    return turnMotor.getSupplyCurrent().getValueAsDouble();
  }

  public boolean isEncoderConnected() {
    return turnEncoder.isConnected();
  }

  public double getRawEncoderValue() {
    return turnEncoder.getPosition().getValueAsDouble();
  }

  public void setSwerveModuleState(SwerveModuleState newState) {
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

  @Override
  public String toString() {
    return moduleName + " Module";
  }

  public boolean isTurnMotorConnected() {
    return turnMotor.isConnected();
  }

  public boolean isDriveMotorConnected() {
    return driveMotor.isConnected();
  }

  // This runs a command to test the connection of each component, then runs another test if the relevent components are connected
  public Command TestConnectionThenModule(
    ErrorGroup errorGroupHandler
  ) {
    return Commands.sequence(
      new TestModuleComponentsConnection(
        errorGroupHandler::addTestSetEntry,
        errorGroupHandler::setTestStatusUsingTestResult,
        this
      ),
      // This Commands.either runs an empty command or a test command based on the result of the command above
      // It checks two different error status' before running the command
      Commands.either(
        Commands.race(
          new TestTurnMotorAndEncoderOnModule(
            errorGroupHandler::addTestSetEntry,
            errorGroupHandler::setTestStatusUsingTestResult,
            this
          ),
          // the command is also in a 5 second time out, because the command takes aprox 4.5
          Commands.waitSeconds(5)
        ),
        Commands.none(),
        () -> errorGroupHandler.getTestStatus("Turn Encoder Not Connected", this) == AlertType.kInfo && 
              errorGroupHandler.getTestStatus("Turn Motor Not Connected", this) == AlertType.kInfo
      )
    );
  }

  public Phoenix6SwerveModuleState getState() {
    return state;
  }
}