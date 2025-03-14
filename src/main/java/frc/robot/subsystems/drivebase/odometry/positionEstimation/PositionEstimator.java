// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase.odometry.positionEstimation;

import java.util.Arrays;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState.Phoenix6DrivebaseState;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState.Phoenix6SwerveModuleState;

public class PositionEstimator {

  public SwerveDriveKinematics swerveDriveKinematics;
  public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  public final Field2d swerveOdometryField2d = new Field2d();

  Phoenix6DrivebaseState drivebaseState;
  Phoenix6SwerveModuleState[] swerveStates;
  BooleanConsumer setPhoenix6OdometryReadLock;
  Drivebase drivebase;

  public ReentrantReadWriteLock stateLock = new ReentrantReadWriteLock(true);

  public PositionEstimator(
    Phoenix6DrivebaseState drivebaseState, 
    Phoenix6SwerveModuleState[] swerveStates,
    BooleanConsumer setPhoenix6OdometryReadLock,
    Translation2d[] moduleLocations,
    Drivebase drivebase
  ) {
    this.drivebaseState = drivebaseState;
    this.swerveStates = swerveStates;
    this.setPhoenix6OdometryReadLock = setPhoenix6OdometryReadLock;
    this.drivebase = drivebase;

    swerveDriveKinematics = new SwerveDriveKinematics(moduleLocations);

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
      swerveDriveKinematics,
      drivebaseState.getGyroAngle(),
      Arrays.stream(swerveStates)
        .map(swerveState -> swerveState.getSwerveModulePosition())
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
    setPhoenix6OdometryReadLock.accept(true);
    for(int i =0; i < 4; i++) {
      swerveDrivePoseEstimator.update(
        drivebaseState.getGyroAngle(),
        new SwerveModulePosition[] {
          swerveStates[0].getSwerveModulePosition(),
          swerveStates[1].getSwerveModulePosition(),
          swerveStates[2].getSwerveModulePosition(),
          swerveStates[3].getSwerveModulePosition()
        }
      );
    }

    swerveOdometryField2d.setRobotPose(
      swerveDrivePoseEstimator.getEstimatedPosition()
    );

    setPhoenix6OdometryReadLock.accept(false);
    stateLock.writeLock().unlock();
  }

  public void reset(Pose2d newPose) {
    stateLock.writeLock().lock();
    this.drivebase.resetGyroHeading(newPose.getRotation());
    swerveDrivePoseEstimator.resetPosition(
      drivebaseState.getGyroAngle(),
      Arrays.stream(swerveStates)
        .map(state -> state.getSwerveModulePosition())
        .toArray(SwerveModulePosition[]::new),
      newPose
    );
    stateLock.writeLock().unlock();
  }

  public void pathplannerReset(Pose2d newPose) {
    System.out.println("pathplannerReset x pos: " + newPose.getX());
    System.out.println("pathplannerReset y pos: " + newPose.getY());
    System.out.println("pathplannerReset theta pos: " + newPose.getRotation().getDegrees());
    reset(newPose);
  }

  public ReentrantReadWriteLock.ReadLock getReadLock() {
    return stateLock.readLock();
  }

  public Pose2d getPose() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  /**
   * Called only in the vision class' periodic method.
   */
  public void addVisionMeasurement(
    Pose2d estimatedPose,
    double timestamp,
    Matrix<N3, N1> stdDevs
  ) {
    stateLock.writeLock().lock();
    swerveDrivePoseEstimator.addVisionMeasurement(
      estimatedPose, timestamp, stdDevs
    );
    swerveOdometryField2d.setRobotPose(
      swerveDrivePoseEstimator.getEstimatedPosition()
    );
    stateLock.writeLock().unlock();
  }

}
