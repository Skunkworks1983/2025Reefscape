// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.utils.error.ErrorGroup;
import frc.robot.utils.error.DiagnosticSubsystem;

public class Drivebase extends SubsystemBase implements DiagnosticSubsystem {

  private SwerveModule swerveModules[] = new SwerveModule[Constants.Drivebase.MODULES.length];
  //TODO private Pigeon2 gyro = new Pigeon2(26, Constants.Drivebase.CANIVORE_NAME);
  private StructArrayPublisher<SwerveModuleState> publisher1 = NetworkTableInstance.getDefault().getStructArrayTopic("Desired swervestates", SwerveModuleState.struct).publish();
  private StructArrayPublisher<SwerveModuleState> publisher2 = NetworkTableInstance.getDefault().getStructArrayTopic("Actual swervestates", SwerveModuleState.struct).publish();


  private SwerveDriveKinematics swerveDriveKinematics;

  public Drivebase() {
    Translation2d[] moduleLocations = new Translation2d[Constants.Drivebase.MODULES.length];

    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i] = new SwerveModule(Constants.Drivebase.MODULES[i]);
      moduleLocations[i] = swerveModules[i].moduleLocation;
    }

    Pigeon2Configuration gConfiguration = new Pigeon2Configuration();
    gConfiguration.MountPose.MountPoseYaw = 0; 
    //TODO gyro.getConfigurator().apply(gConfiguration);
    //TODO resetGyroHeading();

    swerveDriveKinematics = new SwerveDriveKinematics(moduleLocations);
  }

  @Override
  public void periodic() {}

  // TODO: add docstring
  private void drive(double xMetersPerSecond, double yMetersPerSecond,
    double degreesPerSecond, boolean isFieldRelative) {
    ChassisSpeeds chassisSpeeds;
    double radiansPerSecond = Units.degreesToRadians(degreesPerSecond);
    if (isFieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond,
        radiansPerSecond, getGyroAngle());
    } 
    else {
      chassisSpeeds = new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, radiansPerSecond);
    }

    SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, 
      Constants.Drivebase.Info.MAX_MODULE_SPEED
    );
    setModuleStates(swerveModuleStates);
  }

  private void setModuleStates(SwerveModuleState[] moduleStates) {

    publisher1.set(moduleStates);
    publisher2.set(getSwerveModuleStates());
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i].setSwerveModulState(moduleStates[i]);
    }
  }

  public void resetGyroHeading() {
    //TODO gyro.setYaw(0);
  }

  public void setAllDriveMotorBreakMode(boolean breakMode) {
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i].setBrakeMode(breakMode);
    }
  }

  // rotation from gyro is counterclockwise positive while we need clockwise positive
  private Rotation2d getGyroAngle() {
    //TODO return Rotation2d.fromDegrees(-gyro.getYaw().getValueAsDouble());
    return new Rotation2d(0.0);
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {

    SwerveModuleState[] moduleStates = new SwerveModuleState[Constants.Drivebase.MODULES.length];
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      moduleStates[i] = swerveModules[i].getSwerveModuleState();
    }

    return swerveDriveKinematics.toChassisSpeeds(moduleStates);
  }

  private SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[Constants.Drivebase.MODULES.length];
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      moduleStates[i] = swerveModules[i].getSwerveModuleState();
    }
    return moduleStates;
  }

  public void setAllModulesTurnPidActive() {
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i].setTurnControllerActive(true);
    }
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(),
      getGyroAngle());
  }

  public Command getSwerveTeleopCommand(
    DoubleSupplier xMetersPerSecond,
    DoubleSupplier yMetersPerSecond, 
    DoubleSupplier degreesPerSecond,
    boolean isFieldRelative
  ) {
    int fieldOrientationMultiplier;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
      fieldOrientationMultiplier = 1;
    }
    else {
      fieldOrientationMultiplier = -1;
    }
    return runEnd(
      () -> {
        drive(
          xMetersPerSecond.getAsDouble() * fieldOrientationMultiplier,
          yMetersPerSecond.getAsDouble() * fieldOrientationMultiplier,
          degreesPerSecond.getAsDouble(),
          isFieldRelative
        );
      },
      () -> {
        drive(
          0,
          0,
          0,
          isFieldRelative
        );
      }
    ).beforeStarting(
      () -> {
        setAllModulesTurnPidActive();
      }
    );
  }

  @Override
  public Command getErrorCommand(
    ErrorGroup errorGroupHandler
  ) {
    Command[] swerveModuleCommandArray = new Command[Constants.Drivebase.MODULES.length];
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModuleCommandArray[i] = swerveModules[i].TestConnectionThenModule(errorGroupHandler);
    }
    return Commands.parallel(swerveModuleCommandArray);
  }
}
