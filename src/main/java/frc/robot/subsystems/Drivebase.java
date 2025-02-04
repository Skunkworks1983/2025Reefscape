// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

public class Drivebase extends SubsystemBase {

  private AHRS gyro = new AHRS(NavXComType.kUSB1);
  private SwerveModule swerveModules[] = new SwerveModule[Constants.Drivebase.MODULES.length];
  private SwerveDriveKinematics swerveDriveKinematics;
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private final Field2d swerveOdometryField2d = new Field2d();

  Vision vision = new Vision(
      this::addVisionMeasurement,
      new VisionIOPhotonVision(
          VisionConstants.CAMERA_NAMES[0],
          VisionConstants.ROBOT_TO_CAMERA_TRANSFORM));

  // TODO: add vision to drivebase constructor, construct vision in robot.java
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
        new SwerveModulePosition[] {
            swerveModules[0].getSwerveModulePosition(),
            swerveModules[1].getSwerveModulePosition(),
            swerveModules[2].getSwerveModulePosition(),
            swerveModules[3].getSwerveModulePosition()
        },
        new Pose2d());

    SmartDashboard.putData("Swerve Drive Odometry", swerveOdometryField2d);
    swerveOdometryField2d.setRobotPose(new Pose2d());
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  /**
   * Called only in the vision class' periodic method.
   */
  public void addVisionMeasurement(
      Pose2d estimatedPose,
      double timestamp,
      Matrix<N3, N1> stdDevs) {

    swerveDrivePoseEstimator.addVisionMeasurement(estimatedPose, timestamp, stdDevs);
    swerveOdometryField2d.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
  }

  /**
   * Must be called every loop in <code>periodic()</code> to keep odometry up to
   * date.
   */
  public void updateOdometry() {
    swerveDrivePoseEstimator.update(
        getGyroAngle(),
        new SwerveModulePosition[] {
            swerveModules[0].getSwerveModulePosition(),
            swerveModules[1].getSwerveModulePosition(),
            swerveModules[2].getSwerveModulePosition(),
            swerveModules[3].getSwerveModulePosition()
        });

    swerveOdometryField2d.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
  }

  /** Reset the <code>SwerveDrivePoseEstimator</code> to the given pose. */
  public void resetOdometry(Pose2d newPose) {
    swerveDrivePoseEstimator.resetPosition(
        getGyroAngle(),
        Arrays.stream(swerveModules)
            .map(swerveModule -> swerveModule.getSwerveModulePosition())
            .toArray(SwerveModulePosition[]::new),
        newPose);
  }

  // TODO: add docstring
  private void drive(double xMetersPerSecond,
      double yMetersPerSecond, double degreesPerSecond, boolean isFieldRelative) {
    ChassisSpeeds chassisSpeeds;
    double radiansPerSecond = Units.degreesToRadians(degreesPerSecond);
    if (isFieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond,
          radiansPerSecond, getGyroAngle());
    } else {
      chassisSpeeds = new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, radiansPerSecond);
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

  public Rotation2d getGyroAngle() {
    // Negated because gyro measurements are counterclockwise-positive.
    double angleDegrees = -gyro.getAngle();
    SmartDashboard.putNumber("Gyro", angleDegrees);
    return Rotation2d.fromDegrees(angleDegrees);
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
      DoubleSupplier degreesPerSecond,
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
              degreesPerSecond.getAsDouble(),
              isFieldRelative);
        },
        () -> {
          drive(
              0,
              0,
              0,
              isFieldRelative);
        });
  }
}
