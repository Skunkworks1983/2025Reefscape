// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.utils.odometry.Phoenix6Odometry;
import frc.robot.utils.odometry.subsystemSignals.Phoenix6DrivebaseSignal;
import frc.robot.utils.odometry.subsystemSignals.Phoenix6DrivebaseSignal.Field;
import frc.robot.utils.odometry.subsystemState.Phoenix6DrivebaseState;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.utils.error.ErrorGroup;
import frc.robot.utils.error.DiagnosticSubsystem;

public class Drivebase extends SubsystemBase implements DiagnosticSubsystem {

  Phoenix6Odometry phoenix6Odometry = new Phoenix6Odometry();
  private SwerveModule swerveModules[] = new SwerveModule[Constants.Drivebase.MODULES.length];
  private Pigeon2 gyro = new Pigeon2(Constants.Drivebase.PIGEON_ID, Constants.Drivebase.CANIVORE_NAME);
  private StructArrayPublisher<SwerveModuleState> desiredSwervestate = NetworkTableInstance.getDefault().getStructArrayTopic("Desired swervestate", SwerveModuleState.struct).publish();
  private StructArrayPublisher<SwerveModuleState> actualSwervestate = NetworkTableInstance.getDefault().getStructArrayTopic("Actual swervestate", SwerveModuleState.struct).publish();

  public Drivebase() {
    gyro.setYaw(0.0);
    phoenix6Odometry.registerGyroSignal(gyro.getYaw());

    Arrays.stream(swerveModules)
      .forEach(module -> phoenix6Odometry.registerSwerveModule(module));
    phoenix6Odometry.startRunning();

    phoenix6Odometry.stateLock.readLock().lock();
    Pigeon2Configuration gConfiguration = new Pigeon2Configuration();
    gConfiguration.MountPose.MountPoseYaw = 0; 
    gyro.getConfigurator().apply(gConfiguration);
    resetGyroHeading();

    phoenix6Odometry.stateLock.readLock().unlock();
    // Ensure robot code won't crash if the vision subsystem fails to initialize.
    try {
      new Vision(
        this::addVisionMeasurement,
        new VisionIOPhotonVision(VisionConstants.CAMERA_0_NAME, VisionConstants.CAMERA_0_TRANSFORM)
      );
    } catch(Exception exception) {
      System.out.println("Vision subsystem failed to initialize. See the below stacktrace for more details: ");
      exception.printStackTrace();
    }
  }

  @Override
  public void periodic() { }

  /**
   * Called only in the vision class' periodic method.
   */
  public void addVisionMeasurement(
    Pose2d estimatedPose,
    double timestamp,
    Matrix<N3, N1> stdDevs) {

    phoenix6Odometry.stateLock.writeLock().lock();
    phoenix6Odometry.swerveDrivePoseEstimator.addVisionMeasurement(
      estimatedPose, timestamp, stdDevs
    );
    phoenix6Odometry.swerveOdometryField2d.setRobotPose(
      phoenix6Odometry.swerveDrivePoseEstimator.getEstimatedPosition()
    );
    phoenix6Odometry.stateLock.writeLock().unlock();
  }

  /** Reset the <code>SwerveDrivePoseEstimator</code> to the given pose. */
  public void resetOdometry(Pose2d newPose) {
    phoenix6Odometry.stateLock.readLock().lock();
    Phoenix6DrivebaseSignal currentState = phoenix6Odometry.getState();
    phoenix6Odometry.swerveDrivePoseEstimator.resetPosition(
      currentState.getValue(Field.GYRO),
      Arrays.stream(currentState.modules)
        .map(swerveModule -> swerveModule.getSwerveModulePosition())
        .toArray(SwerveModulePosition[]::new),
      newPose
    );
    phoenix6Odometry.stateLock.readLock().unlock();
  }

  // TODO: add docstring
  private void drive(double xMetersPerSecond, double yMetersPerSecond,
    double degreesPerSecond, boolean isFieldRelative
  ) {
    phoenix6Odometry.stateLock.readLock().lock();
    Phoenix6DrivebaseSignal currentState = phoenix6Odometry.getState();
    ChassisSpeeds chassisSpeeds;
    double radiansPerSecond = Units.degreesToRadians(degreesPerSecond);
    if (isFieldRelative) {
      chassisSpeeds = 
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xMetersPerSecond, 
            yMetersPerSecond,
            radiansPerSecond, 
            currentState.getGyroAngle()
          );
    phoenix6Odometry.stateLock.readLock().unlock();
    } else {
      chassisSpeeds = new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, radiansPerSecond);
    }

    SwerveModuleState[] swerveModuleStates = phoenix6Odometry.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates,
      Constants.Drivebase.Info.MAX_MODULE_SPEED
    );
    setModuleStates(swerveModuleStates);
  }

  private void setModuleStates(SwerveModuleState[] moduleStates) {
    desiredSwervestate.set(moduleStates);
    actualSwervestate.set(getSwerveModuleStates());
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i].setSwerveModuleState(moduleStates[i]);
    }
  }

  public void resetGyroHeading() {
    gyro.setYaw(0);
  }

  public void setAllDriveMotorBreakMode(boolean breakMode) {
    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i].setBrakeMode(breakMode);
    }
  }

  @SuppressWarnings("unused")
  private ChassisSpeeds getRobotRelativeSpeeds() {
    Phoenix6DrivebaseState currentState = phoenix6Odometry.getState();
    phoenix6Odometry.stateLock.readLock().lock();
    SwerveModuleState[] moduleStates = new SwerveModuleState[Constants.Drivebase.MODULES.length];
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      moduleStates[i] = currentState.swerveState[i].getSwerveModuleState();
    }
    ChassisSpeeds chassisSpeeds = phoenix6Odometry.swerveDriveKinematics.toChassisSpeeds(moduleStates);
    phoenix6Odometry.stateLock.readLock().unlock();
    return chassisSpeeds;
  }

  private SwerveModuleState[] getSwerveModuleStates() {
    phoenix6Odometry.stateLock.readLock().lock();
    Phoenix6DrivebaseState currentState = phoenix6Odometry.getState();
    SwerveModuleState[] moduleStates = new SwerveModuleState[Constants.Drivebase.MODULES.length];
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      moduleStates[i] = currentState.swerveState[i].getSwerveModuleState();
    }
    phoenix6Odometry.stateLock.readLock().unlock();
    return moduleStates;
  }

  public void setAllModulesTurnPidActive() {
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i].setTurnControllerActive(true);
    }
  }

  public Command getSwerveTeleopCommand(
      DoubleSupplier xMetersPerSecond,
      DoubleSupplier yMetersPerSecond,
      DoubleSupplier degreesPerSecond,
      boolean isFieldRelative) {
    int fieldOrientationMultiplier;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
      fieldOrientationMultiplier = 1;
    } else {
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
