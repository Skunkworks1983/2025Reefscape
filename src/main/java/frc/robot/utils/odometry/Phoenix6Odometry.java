// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.odometry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.Constants;
import frc.robot.utils.SignalValue;

public class Phoenix6Odometry {

  // For information on what a SignalValue is, see the comment in the SignalValue class.
  List<SignalValue<? extends Unit>> 
    SignalValueGroup = new ArrayList<>();
  Thread thread = new Thread(this::run);
  public AtomicBoolean isRunning = new AtomicBoolean(false);
  int failedUpdates;
  int vaildUpdates;
  public ReentrantReadWriteLock stateLock = new ReentrantReadWriteLock();

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
    signalsGroup = new ArrayList<>();
    isRunning.set(true);
    updateMotors();
    updateDrivebaseState();
    signalsGroup.addAll(turnMotorPositionSignalGroup);
    signalsGroup.addAll(turnMotorVelocitySignalGroup);
    signalsGroup.addAll(driveMotorVelocitySignalGroup);
    thread.start();
  }

  public void run() {
    while(isRunning.get()) {
      updateMotors();
      updateDrivebaseState();
    }
  }

  public void updateMotors(){
    // Notes: waitForAll uses signals as an out param.
    StatusCode status = BaseStatusSignal.waitForAll(
      Constants.Phoenix6Odometry.updatesPerSecond,
      (BaseStatusSignal[])signalsGroup.toArray()
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

    Phoenix6SwerveModuleState currentSwerveModuleStates[] = new Phoenix6SwerveModuleState[Constants.Drivebase.MODULES.length];

    assert driveMotorVelocitySignalGroup.size() == turnMotorPositionSignalGroup.size();
    assert turnMotorVelocitySignalGroup.size() == turnMotorPositionSignalGroup.size();
    for (int i = 0; i < driveMotorVelocitySignalGroup.size(); i++) {
      currentSwerveModuleStates[i] = new Phoenix6SwerveModuleState(
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(
          driveMotorPositionSignalGroup.get(i),
          driveMotorVelocitySignalGroup.get(i)
        ),
        driveMotorVelocitySignalGroup.get(i).getValueAsDouble(),
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(
          turnMotorPositionSignalGroup.get(i),
          turnMotorVelocitySignalGroup.get(i))
        );
      }

    Phoenix6DrivebaseState currentPhoenix6DrivebaseState = new Phoenix6DrivebaseState(
      // Negated because gyro measurements are counterclockwise-positive.
      Rotation2d.fromDegrees(-gyroStatusSignal.getValueAsDouble()),
      currentSwerveModuleStates
    );

    phoenix6DrivebaseState = currentPhoenix6DrivebaseState;
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
    driveMotorVelocitySignalGroup.set(moduleNumber, driveMotorVelocity);
  }
}
