// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Drivebase extends SubsystemBase {

  private AHRS gyro = new AHRS(NavXComType.kUSB1);
  private SwerveModule swerveModules[] = new SwerveModule[Constants.Drivebase.MODULES.length];
  private SwerveDriveKinematics swerveDriveKinematics;
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private final Field2d drivebaseOdometryField2d = new Field2d();

  public Drivebase() {
    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i] = new SwerveModule(Constants.Drivebase.MODULES[i]);
    }

    swerveDriveKinematics = new SwerveDriveKinematics(
        swerveModules[0].moduleLocation,
        swerveModules[1].moduleLocation,
        swerveModules[2].moduleLocation,
        swerveModules[3].moduleLocation);

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
        swerveDriveKinematics,
        getGyroAngle(),
        new SwerveModulePosition[] {new SwerveModulePosition()}, new Pose2d());
        // Arrays.stream(swerveModules)
        //     .map(swerveModule -> swerveModule.getSwerveModulePosition())
        //     .toArray(SwerveModulePosition[]::new),
        // new Pose2d());

      SmartDashboard.putData("Drivebase Odometry", drivebaseOdometryField2d);
      drivebaseOdometryField2d.setRobotPose(new Pose2d());
  }

  public void addVisionMeasurement(
      Pose2d estimatedPose,
      double timestamp,
      Matrix<N3, N1> stdDevs) {

      swerveDrivePoseEstimator.addVisionMeasurement(estimatedPose, timestamp, stdDevs);
      drivebaseOdometryField2d.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
  }

  @Override
  public void periodic() {
    System.out.println("Drivebase Periodic Running");
  }

  // TODO: add docstring
  private void drive(double xMetersPerSecond,
      double yMetersPerSecond, Rotation2d rotationsPerSecond, boolean isFieldRelative) {
    ChassisSpeeds chassisSpeeds;
    if (isFieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond,
          rotationsPerSecond.getRadians(), getGyroAngle());
    } else {
      chassisSpeeds = new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, rotationsPerSecond.getRadians());
    }

    SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates,
        Constants.Drivebase.Info.MAX_MODULE_SPEED);
    setModuleStates(swerveModuleStates);
  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {
    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i].setSwerveModulState(moduleStates[i]);
    }
  }

  public void setGyroHeading(Rotation2d newHeading) {
    gyro.setAngleAdjustment(newHeading.getDegrees());
  }

  public void setAllDriveMotorBreakMode(boolean breakMode) {
    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i].setBrakeMode(breakMode);
    }
  }

  // rotation from gyro is counterclockwise positive while we need clockwise
  // positive
  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveDriveKinematics.toChassisSpeeds(
        swerveModules[0].getSwerveModuleState(),
        swerveModules[1].getSwerveModuleState(),
        swerveModules[2].getSwerveModuleState(),
        swerveModules[3].getSwerveModuleState());
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(),
        getGyroAngle());
  }

  public Command getSwerveTeleopCommand(
      DoubleSupplier xMetersPerSecond,
      DoubleSupplier yMetersPerSecond,
      Supplier<Rotation2d> rotationPerSecond,
      boolean isFieldRelative) {
    int fieldOrientationMultiplier;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
      fieldOrientationMultiplier = 1;
    } else {
      fieldOrientationMultiplier = -1;
    }
    return Commands.runEnd(
        () -> {
          drive(
              xMetersPerSecond.getAsDouble() * fieldOrientationMultiplier,
              yMetersPerSecond.getAsDouble() * fieldOrientationMultiplier,
              rotationPerSecond.get(),
              isFieldRelative);
        },
        () -> {
          drive(
              0,
              0,
              Rotation2d.kZero,
              isFieldRelative);
        });
  }
}
