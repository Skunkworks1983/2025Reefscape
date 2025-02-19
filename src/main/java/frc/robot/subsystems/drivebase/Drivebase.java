// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.subsystems.drivebase.odometry.OdometryThread;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.Phoenix6Odometry;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState.Phoenix6DrivebaseState;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState.Phoenix6SwerveModuleState;
import frc.robot.subsystems.drivebase.odometry.positionEstimation.PositionEstimator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.utils.error.ErrorGroup;
import frc.robot.utils.error.DiagnosticSubsystem;

public class Drivebase extends SubsystemBase implements DiagnosticSubsystem {

  private OdometryThread odometryThread;
  private Phoenix6Odometry phoenix6Odometry = new Phoenix6Odometry();
  private PositionEstimator positionEstimator;
  final private Phoenix6DrivebaseState state;

  private SwerveModule swerveModules[] = new SwerveModule[Constants.Drivebase.MODULES.length];
  private Pigeon2 gyro = new Pigeon2(Constants.Drivebase.PIGEON_ID, Constants.Drivebase.CANIVORE_NAME);
  private StructArrayPublisher<SwerveModuleState> desiredSwervestate = NetworkTableInstance.getDefault().getStructArrayTopic("Desired swervestate", SwerveModuleState.struct).publish();
  private StructArrayPublisher<SwerveModuleState> actualSwervestate = NetworkTableInstance.getDefault().getStructArrayTopic("Actual swervestate", SwerveModuleState.struct).publish();

  public Drivebase() {
    resetGyroHeading();
    state = phoenix6Odometry.registerDrivebase(gyro);

    Translation2d[] moduleLocations = new Translation2d[Constants.Drivebase.MODULES.length];

    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i] = new SwerveModule(Constants.Drivebase.MODULES[i], phoenix6Odometry);
      moduleLocations[i] = swerveModules[i].moduleLocation;
    }

    // Constructs a pose estimator with this state, and the state of the swerve modules.
    positionEstimator = new PositionEstimator(
      this.getState(), 
      Arrays.stream(swerveModules)
        .map(module -> module.getState())
        .toArray(Phoenix6SwerveModuleState[]::new),
      phoenix6Odometry::setReadLock,
      moduleLocations
    );


    odometryThread = new OdometryThread(phoenix6Odometry, positionEstimator);
    odometryThread.startThread();

    Pigeon2Configuration gConfiguration = new Pigeon2Configuration();
    gConfiguration.MountPose.MountPoseYaw = 0; 
    gyro.getConfigurator().apply(gConfiguration);
    resetGyroHeading();

    // Ensure robot code won't crash if the vision subsystem fails to initialize.
    try {
      new Vision(
        positionEstimator::addVisionMeasurement,
        new VisionIOPhotonVision(
          VisionConstants.FRONT_CAMERA_NAME, 
          VisionConstants.ROBOT_TO_FRONT_CAMERA
        ),
        new VisionIOPhotonVision(
          VisionConstants.SIDE_CAMERA_NAME, 
          VisionConstants.ROBOT_TO_SIDE_CAMERA
        )
      );
    } catch(Exception exception) {
      System.out.println("Vision subsystem failed to initialize. See the below stacktrace for more details: ");
      exception.printStackTrace();
    }
  }

  @Override
  public void periodic() {}

  // TODO: add docstring
  private void drive(double xMetersPerSecond, double yMetersPerSecond,
    double degreesPerSecond, boolean isFieldRelative
  ) {
    getState().getReadLock().lock();
    ChassisSpeeds chassisSpeeds;
    double radiansPerSecond = Units.degreesToRadians(degreesPerSecond);
    if (isFieldRelative) {
      chassisSpeeds = 
        ChassisSpeeds.fromFieldRelativeSpeeds(
          xMetersPerSecond, 
          yMetersPerSecond,
          radiansPerSecond, 
          state.getGyroAngle()
        );
    getState().getReadLock().unlock();
    } else {
      chassisSpeeds = new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, radiansPerSecond);
    }
    positionEstimator.getReadLock().lock();
    SwerveModuleState[] swerveModuleStates = 
      positionEstimator.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    positionEstimator.getReadLock().unlock();
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
    positionEstimator.stateLock.readLock().lock();
    SwerveModuleState[] moduleStates = new SwerveModuleState[Constants.Drivebase.MODULES.length];
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      moduleStates[i] = swerveModules[i].getState().getSwerveModuleState();
    }
    ChassisSpeeds chassisSpeeds = positionEstimator.swerveDriveKinematics.toChassisSpeeds(moduleStates);
    positionEstimator.stateLock.readLock().unlock();
    return chassisSpeeds;
  }

  private SwerveModuleState[] getSwerveModuleStates() {
    phoenix6Odometry.setReadLock(true);
    SwerveModuleState[] moduleStates = new SwerveModuleState[Constants.Drivebase.MODULES.length];
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      moduleStates[i] = swerveModules[i].getState().getSwerveModuleState();
    }
    phoenix6Odometry.setReadLock(false);
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
        drive(0, 0, 0, isFieldRelative);
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

  public Phoenix6DrivebaseState getState() {
    return state;
  }
}
