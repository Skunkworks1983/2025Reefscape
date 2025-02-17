// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.odometry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.odometry.subsystemSignals.Phoenix6DrivebaseSignal;
import frc.robot.utils.odometry.subsystemState.Phoenix6DrivebaseState;
import frc.robot.utils.odometry.subsystemState.Phoenix6SwerveModuleState;
import frc.robot.utils.odometry.subsystemSignals.Phoenix6SwerveModuleSignal;
import frc.robot.utils.odometry.subsystemSignals.SubsystemSignal;

public class Phoenix6Odometry {


  // Odometry:
  public SwerveDriveKinematics swerveDriveKinematics;
  // pose estimator depends on swerve drive kinematics
  public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  // field 2d depends on pose estimator
  public final Field2d swerveOdometryField2d = new Field2d();

  Phoenix6DrivebaseSignal drivebase;
  List<SubsystemSignal> subsystems = List.of(drivebase);

  public AtomicBoolean isRunning = new AtomicBoolean(false);
  int failedUpdates = 0;
  int vaildUpdates = 0;

  public ReentrantReadWriteLock stateLock;

  private List<BaseStatusSignal> getAllSignals() {
    List<BaseStatusSignal> allSignals = new ArrayList<>();
    subsystems.stream()
      .map(subsystem -> subsystem.getSignals())
      .forEach(
        subsystemSignals -> allSignals.addAll(subsystemSignals)
      );
      return allSignals;
  }

  Thread thread = new Thread(
    () -> { 
      while(isRunning.get()) update();
    }
  );

  public void startRunning() {

    /*
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
    */

    isRunning.set(true); //isRunning is set first so signals can't be added

    // Using toArray(new BaseStatusSignal[0]) to specify type to be used.
    BaseStatusSignal.setUpdateFrequencyForAll(
      Constants.Phoenix6Odometry.updatesPerSecond,
      getAllSignals().toArray(new BaseStatusSignal[0])
    );
    update(); // Run thread once so that it will never have null/incorrect values.
    thread.start();
  }


  /**
   * Must be called every loop in <code>periodic()</code> to keep odometry up to
   * date.
   */
  public void updateOdometry() {
    for(int i =0; i < 4; i++){
      swerveDrivePoseEstimator.update(
        getDrivebaseState().getGyroAngle(),
        new SwerveModulePosition[] {
          getSwerveModuleState(0).getSwerveModulePosition(),
          getSwerveModuleState(1).getSwerveModulePosition(),
          getSwerveModuleState(2).getSwerveModulePosition(),
          getSwerveModuleState(3).getSwerveModulePosition()
        }
      );
    }
    swerveOdometryField2d.setRobotPose(
      swerveDrivePoseEstimator.getEstimatedPosition()
    );
  }

  // called on a thread
  public void update() {
    updateSignals(); // Get most recent data on StatusSignals
    updateState(); // Record/calculate values recived from previous step.
  }

  public void updateSignals() {
    // Note: waitForAll uses signals as an out param.
    // Using toArray(new BaseStatusSignal[0]) to specify type to be used.
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
    stateLock.writeLock().lock();

    BaseStatusSignal.refreshAll(
      getAllSignals().toArray(new BaseStatusSignal[0])
    );

    drivebase.updateCachedValues();

    stateLock.writeLock().unlock();
  }

  public void registerGyroSignal(Pigeon2 gyro) {
    registerGyroSignal(gyro.getYaw());
  }

  public void registerGyroSignal(StatusSignal<Angle> gyroAngleSignal) {
    if(isRunning.get()) {
      throw new RuntimeException("Can not add more signals while running!");
    }
    new Phoenix6DrivebaseSignal(
      gyroAngleSignal
    );
  }

  public Phoenix6SwerveModuleSignal registerSwerveModule(int moduleNumber, SwerveModule swerveModule){
    return registerSwerveModule (
      0,
      swerveModule.turnMotorPositionSignal,
      swerveModule.turnMotorVelocitySignal,
      swerveModule.driveMotorPositionSignal,
      swerveModule.driveMotorVelocitySignal
    );
  }

  public Phoenix6SwerveModuleSignal registerSwerveModule (
    int moduleNumber,
    StatusSignal<Angle> turnMotorPositionSignal, 
    StatusSignal<AngularVelocity> turnMotorVelocitySignal, 
    StatusSignal<Angle> driveMotorPositionSignal, 
    StatusSignal<AngularVelocity> driveMotorVelocitySignal
    // TODO: check if drive motor acceleration exists
  ) {

    if(isRunning.get()) {
      throw new RuntimeException("Can not add more signals while running!");
    }
    if(moduleNumber >= Constants.Drivebase.MODULES.length) {
      throw new RuntimeException("moduleNumber can not be greater or equal to the number of modules on the robot");
    }
    Phoenix6SwerveModuleSignal module = new Phoenix6SwerveModuleSignal(
      new SignalValue(turnMotorPositionSignal), 
      new SignalValue(turnMotorPositionSignal),
      new SignalValue(turnMotorPositionSignal), 
      new SignalValue(turnMotorPositionSignal)
    );

    return drivebase.modules[moduleNumber] = module;
  }

  public Phoenix6DrivebaseState getDrivebaseState() {
    return drivebase.getState();
  }

  public Phoenix6SwerveModuleState getSwerveModuleState(int index) {
    return drivebase.modules[index].getState();
  }
}
