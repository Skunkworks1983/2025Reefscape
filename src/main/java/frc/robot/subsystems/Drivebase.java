// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Drivebase extends SubsystemBase {

  private SwerveModule swerveModules[] = new SwerveModule[Constants.Drivebase.MODULES.length];
  private AHRS gyro = new AHRS(NavXComType.kUSB1);

  private SwerveDriveKinematics swerveDriveKinematics;

  public Drivebase() {
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i] = new SwerveModule(Constants.Drivebase.MODULES[i]);
    }
    swerveDriveKinematics = new SwerveDriveKinematics(
      swerveModules[0].moduleLocation,
      swerveModules[1].moduleLocation, 
      swerveModules[2].moduleLocation, 
      swerveModules[3].moduleLocation
    );
  }

  @Override
  public void periodic() {}

  // TODO: add docstring
  private void drive(double xMetersPerSecond,
  double yMetersPerSecond, Rotation2d rotationsPerSecond, boolean isFieldRelative) {
    ChassisSpeeds chassisSpeeds;
    if (isFieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond,
        rotationsPerSecond.getRadians(), getGyroAngle());
    } 
    else {
      chassisSpeeds = new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, rotationsPerSecond.getRadians());
    }

    SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, 
      Constants.Drivebase.Info.MAX_MODULE_SPEED
    );
    setModuleStates(swerveModuleStates);
  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i].setSwerveModulState(moduleStates[i]);
    }
  }

  public void setGyroHeading(Rotation2d newHeading) {
    gyro.setAngleAdjustment(newHeading.getDegrees());
  }

  public void setAllDriveMotorBreakMode(boolean breakMode) {
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i].setBrakeMode(breakMode);
    }
  }

  // rotation from gyro is counterclockwise positive while we need clockwise positive
  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveDriveKinematics.toChassisSpeeds(
      swerveModules[0].getSwerveModuleState(),
      swerveModules[1].getSwerveModuleState(), 
      swerveModules[2].getSwerveModuleState(), 
      swerveModules[3].getSwerveModuleState()
    );
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(),
      getGyroAngle());
  }

  public Command getSwerveTeleopCommand(
    DoubleSupplier xMetersPerSecond,
    DoubleSupplier yMetersPerSecond, 
    Supplier<Rotation2d> rotationPerSecond,
    boolean isFieldRelative
  ) {
    int fieldOrientationMultiplier;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
      fieldOrientationMultiplier = 1;
    }
    else {
      fieldOrientationMultiplier = -1;
    }
    return Commands.runEnd(
      () -> {
        drive(
          xMetersPerSecond.getAsDouble() * fieldOrientationMultiplier,
          yMetersPerSecond.getAsDouble() * fieldOrientationMultiplier,
          rotationPerSecond.get(),
          isFieldRelative
        );
      },
      () -> {
        drive(
          0,
          0,
          Rotation2d.kZero,
          isFieldRelative
        );
      }
    );
  }
}
