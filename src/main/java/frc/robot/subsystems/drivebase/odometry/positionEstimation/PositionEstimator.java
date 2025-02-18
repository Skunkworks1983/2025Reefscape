// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase.odometry.positionEstimation;

import java.util.Arrays;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState.Phoenix6DrivebaseState;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState.Phoenix6SwerveModuleState;

/** Add your docs here. */
public class PositionEstimator {

  Phoenix6DrivebaseState drivebaseState;
  Phoenix6SwerveModuleState[] swerveState;

  public ReentrantReadWriteLock stateLock = new ReentrantReadWriteLock(true);

  public PositionEstimator(
    Phoenix6DrivebaseState drivebaseState, 
    Phoenix6SwerveModuleState[] swerveState
  ) {
    this.drivebaseState = drivebaseState;
    this.swerveState = swerveState;

  }

  // Odometry:
  public SwerveDriveKinematics swerveDriveKinematics;
  // pose estimator depends on swerve drive kinematics
  public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  // field 2d depends on pose estimator
  public final Field2d swerveOdometryField2d = new Field2d();

  
  public void start() {
    Translation2d[] moduleLocations = new Translation2d[Constants.Drivebase.MODULES.length];

    swerveDriveKinematics = new SwerveDriveKinematics(moduleLocations);

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
      swerveDriveKinematics,
      drivebaseState.getGyroAngle(),
      Arrays.stream(swerveState)
        .map(swerveModule -> swerveModule.getSwerveModulePosition())
        .toArray(SwerveModulePosition[]::new),
      new Pose2d()
    );

    SmartDashboard.putData("Swerve Drive Odometry", swerveOdometryField2d);
    swerveOdometryField2d.setRobotPose(new Pose2d());
  }

  /**
   * Must be called every loop in <code>periodic()</code> to keep odometry up to
   * date.
   */
  public void update() {
    stateLock.writeLock().lock();
    for(int i =0; i < 4; i++){
      swerveDrivePoseEstimator.update(
        drivebaseState.getGyroAngle(),
        new SwerveModulePosition[] {
          swerveState[0].getSwerveModulePosition(),
          swerveState[1].getSwerveModulePosition(),
          swerveState[2].getSwerveModulePosition(),
          swerveState[3].getSwerveModulePosition()
        }
      );
    }

    swerveOdometryField2d.setRobotPose(
      swerveDrivePoseEstimator.getEstimatedPosition()
    );
  stateLock.writeLock().unlock();
  }
}
