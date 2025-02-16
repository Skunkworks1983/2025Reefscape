// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.odometry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;

public class Phoenix6Odometry {


  public SwerveDriveKinematics swerveDriveKinematics;
  // pose estimator depends on swerve drive kinematics
  public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  // field 2d depends on pose estimator
  public final Field2d swerveOdometryField2d = new Field2d();





  Thread thread = new Thread(this::run);
  public AtomicBoolean isRunning = new AtomicBoolean(false);
  int failedUpdates;
  int vaildUpdates;

  public ReentrantReadWriteLock stateLock;

  volatile private Phoenix6DrivebaseState phoenix6DrivebaseState;

  List<BaseStatusSignal> signalsGroup;

  StatusSignal<Angle> gyroStatusSignal;
  List<StatusSignal<Angle>> turnMotorPositionSignalGroup = 
    new ArrayList<>(
      Collections.nCopies(Constants.Drivebase.MODULES.length, null)
    ); 

  List<StatusSignal<AngularVelocity>> turnMotorVelocitySignalGroup = 
    new ArrayList<>(
      Collections.nCopies(Constants.Drivebase.MODULES.length, null)
    ); 
  List<StatusSignal<Angle>> driveMotorPositionSignalGroup =
    new ArrayList<>(
      Collections.nCopies(Constants.Drivebase.MODULES.length, null)
    ); 
  List<StatusSignal<AngularVelocity>> driveMotorVelocitySignalGroup =
    new ArrayList<>(
      Collections.nCopies(Constants.Drivebase.MODULES.length, null)
    ); 

  public void startRunning(){

    Translation2d[] moduleLocations = new Translation2d[Constants.Drivebase.MODULES.length];

    swerveDriveKinematics = new SwerveDriveKinematics(moduleLocations);

    Phoenix6DrivebaseState startingState = this.getState();
    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
      swerveDriveKinematics,
      startingState.gyroAngle,
      Arrays.stream(startingState.swerveState)
        .map(swerveModule -> swerveModule.getSwerveModulePosition())
        .toArray(SwerveModulePosition[]::new),
      new Pose2d()
    );

    SmartDashboard.putData("Swerve Drive Odometry", swerveOdometryField2d);
    swerveOdometryField2d.setRobotPose(new Pose2d());
    


    signalsGroup = new ArrayList<>();
    isRunning.set(true); //isRunning means that signals can no longer be added
    updateMotors();
    updateDrivebaseState();
    signalsGroup.addAll(turnMotorPositionSignalGroup);
    signalsGroup.addAll(turnMotorVelocitySignalGroup);
    signalsGroup.addAll(driveMotorVelocitySignalGroup);
    signalsGroup.addAll(driveMotorPositionSignalGroup);
    signalsGroup.add(gyroStatusSignal);

    // Using toArray(new BaseStatusSignal[0]) to specify type to be used.
    BaseStatusSignal.setUpdateFrequencyForAll(
      Constants.Phoenix6Odometry.updatesPerSecond,
      signalsGroup.toArray(new BaseStatusSignal[0])
    );
    thread.start();
  }


  /**
   * Must be called every loop in <code>periodic()</code> to keep odometry up to
   * date.
   */
  public void updateOdometry() {
    Phoenix6DrivebaseState currentState = this.getState();
    for(int i =0; i < 4; i++){
      // logs as velocity
      SmartDashboard.putNumber(
        "modulePosition" + i,
        currentState.swerveState[i].getSwerveModulePosition().distanceMeters);
    }

    swerveDrivePoseEstimator.update(
      currentState.gyroAngle,
      new SwerveModulePosition[] {
        currentState.swerveState[0].getSwerveModulePosition(),
        currentState.swerveState[1].getSwerveModulePosition(),
        currentState.swerveState[2].getSwerveModulePosition(),
        currentState.swerveState[3].getSwerveModulePosition()
      }
    );

    swerveOdometryField2d.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
  }

  public void run() {
    while(isRunning.get()) {
      updateMotors();
      updateDrivebaseState();
    }
  }

  public void updateMotors() {
    // waitForAll uses signals as an out param.
    // Using toArray(new BaseStatusSignal[0]) to specify type to be used.
    StatusCode status = BaseStatusSignal.waitForAll(
      1.0 / Constants.Phoenix6Odometry.updatesPerSecond,
      signalsGroup.toArray(new BaseStatusSignal[0])
    );

    if(status.isOK()) {
      vaildUpdates++;
    } else {
      failedUpdates++;
    }
  }

  public Phoenix6DrivebaseState getState() {
    return phoenix6DrivebaseState;
  }

  private void updateDrivebaseState() {
    stateLock.writeLock().lock();
    Phoenix6SwerveModuleState currentSwerveModuleStates[] 
      = new Phoenix6SwerveModuleState[Constants.Drivebase.MODULES.length];

    assert driveMotorVelocitySignalGroup.size() == turnMotorPositionSignalGroup.size();
    assert turnMotorVelocitySignalGroup.size() == turnMotorPositionSignalGroup.size();
    BaseStatusSignal.refreshAll(
      signalsGroup.toArray(new BaseStatusSignal[0])
    );

    for (int i = 0; i < driveMotorVelocitySignalGroup.size(); i++) {
      currentSwerveModuleStates[i] = new Phoenix6SwerveModuleState(
          driveMotorPositionSignalGroup.get(i).getValueAsDouble(),
          driveMotorVelocitySignalGroup.get(i).getValueAsDouble(),
          turnMotorPositionSignalGroup.get(i).getValueAsDouble()
      );
    }

    Phoenix6DrivebaseState currentPhoenix6DrivebaseState = new Phoenix6DrivebaseState(
      Rotation2d.fromDegrees(gyroStatusSignal.getValueAsDouble()),
      currentSwerveModuleStates
    );

    phoenix6DrivebaseState = currentPhoenix6DrivebaseState;
    stateLock.writeLock().unlock();
  }

  public void registerGyroSignal (StatusSignal<Angle> gyroAngleSignal) {
    if(isRunning.get()) {
      throw new RuntimeException("Can not add more signals while running!");
    }
    gyroStatusSignal = gyroAngleSignal;
  }

  public void registerSwerveModuleSignal (
    int moduleNumber,
    StatusSignal<Angle> turnMotorPositionSignal, 
    StatusSignal<AngularVelocity> turnMotorVelocitySignal, 
    StatusSignal<Angle> driveMotorPosition, 
    StatusSignal<AngularVelocity> driveMotorVelocity
    // TODO: check if drive motor acceleration exists
  ) {
    if(isRunning.get()) {
      throw new RuntimeException("Can not add more signals while running!");
    }
    if(moduleNumber >= Constants.Drivebase.MODULES.length) {
      throw new RuntimeException("moduleNumber can not be greater or equal to the number of modules on the robot");
    }

    turnMotorPositionSignalGroup.set(moduleNumber, turnMotorPositionSignal);
    turnMotorVelocitySignalGroup.set(moduleNumber, turnMotorVelocitySignal);
    driveMotorPositionSignalGroup.set(moduleNumber, driveMotorPosition);
    driveMotorVelocitySignalGroup.set(moduleNumber, driveMotorVelocity);
  }
}
