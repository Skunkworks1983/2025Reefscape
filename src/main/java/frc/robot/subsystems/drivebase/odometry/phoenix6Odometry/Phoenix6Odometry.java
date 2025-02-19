// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase.odometry.phoenix6Odometry;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.constants.Constants;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemSignals.Phoenix6DrivebaseSignal;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemSignals.Phoenix6SwerveModuleSignal;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemSignals.SubsystemSignal;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState.Phoenix6DrivebaseState;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState.Phoenix6SwerveModuleState;

/*
 * Phoenix6Odometry is used to manage the many StatusSignals in the robot.
 * It can be used to register subsystems that contain Pheonix devices, update
 * the signals, set read locks, and update the state the values.
 */
public class Phoenix6Odometry {

  List<SubsystemSignal<?>> subsystemSignals = new LinkedList<>();

  Phoenix6DrivebaseState drivebaseState;
  List<Phoenix6SwerveModuleState> swerveModuleState = new LinkedList<>();

  int failedUpdates = 0;
  int vaildUpdates = 0;

  private List<BaseStatusSignal> getAllSignals() {
    List<BaseStatusSignal> allSignals = new ArrayList<>();

    for(SubsystemSignal<?> subsystem : subsystemSignals) {
      List<BaseStatusSignal> signals = subsystem.getSignals();
      allSignals.addAll(signals);
    }

    return allSignals;
  }

  // Called on a thread
  public void update() {
    updateSignals(); // Force motors to record sensor values
    updateState(); // Record/calculate values recived from previous step
  }

  public void updateSignals() {
    // Note: waitForAll uses signals as an out param.
    // Using toArray(new BaseStatusSignal[0]) to specify type to be used
    StatusCode status = BaseStatusSignal.waitForAll(
      1.0 / Constants.Phoenix6Odometry.updatesPerSecond,
      getAllSignals().toArray(new BaseStatusSignal[0])
    );

    if(status.isOK()) {
      vaildUpdates++;
    } else {
      failedUpdates++;
    }
  }

  private void updateState() {
    setWriteLock(true);

    BaseStatusSignal.refreshAll(
      getAllSignals().toArray(new BaseStatusSignal[0])
    );

    subsystemSignals.forEach(subsystemSignal -> subsystemSignal.updateCachedValues());
    setWriteLock(false);
  }

  public Phoenix6DrivebaseState registerDrivebase(Pigeon2 gyro) {
    Phoenix6DrivebaseSignal drivebaseSignal = 
      new Phoenix6DrivebaseSignal(
        new SignalValue(gyro.getYaw(), gyro.getAngularVelocityZDevice(), 1.0)
      );

    subsystemSignals.add(drivebaseSignal);

    resetUpdateFrequency();
    return drivebaseSignal.getState();
  }

  // turnMotorAccelerationSignal does not exist for current encoders 
  public Phoenix6SwerveModuleState registerSwerveModule (
    CANcoder turnEncoder, TalonFX driveMotor
  ) {
    Phoenix6SwerveModuleSignal moduleSignal = new Phoenix6SwerveModuleSignal(
      new SignalValue(
        turnEncoder.getPosition(), 
        turnEncoder.getVelocity(),
        1.0
      ),
      new SignalValue(
        turnEncoder.getVelocity(),
        1.0
      ),
      new SignalValue(
        driveMotor.getPosition(), 
        driveMotor.getVelocity(), 
        Constants.Drivebase.Info.METERS_PER_REV
      ), 
      new SignalValue(
        driveMotor.getVelocity(), 
        driveMotor.getAcceleration(), 
        Constants.Drivebase.Info.METERS_PER_REV
      )
    );

    subsystemSignals.add(moduleSignal);
    resetUpdateFrequency();

    return moduleSignal.getState();
  }

  private void resetUpdateFrequency() {
    BaseStatusSignal.setUpdateFrequencyForAll(
      Constants.Phoenix6Odometry.updatesPerSecond,
      getAllSignals().toArray(new BaseStatusSignal[0])
    );
  }

  private void setWriteLock(boolean locked) {
    subsystemSignals.forEach(
      subsystemSignal -> {
        if(locked) {
          subsystemSignal.getLock().writeLock().lock();
        } else {
          subsystemSignal.getLock().writeLock().unlock();
        }
      }
    );
  }

  public void setReadLock(boolean locked) {
    subsystemSignals.forEach(
      subsystemSignal -> {
        if(locked) {
          subsystemSignal.getLock().readLock().lock();
        } else {
          subsystemSignal.getLock().readLock().unlock();
        }
      }
    );
  }
}
