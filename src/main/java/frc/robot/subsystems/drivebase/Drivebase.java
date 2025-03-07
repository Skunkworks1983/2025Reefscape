// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.Constants.Drivebase.TeleopFeature;
import frc.robot.subsystems.drivebase.odometry.OdometryThread;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.Phoenix6Odometry;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState.Phoenix6DrivebaseState;
import frc.robot.subsystems.drivebase.odometry.phoenix6Odometry.subsystemState.Phoenix6SwerveModuleState;
import frc.robot.subsystems.drivebase.odometry.positionEstimation.PositionEstimator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.error.ErrorGroup;
import frc.robot.utils.error.DiagnosticSubsystem;

public class Drivebase extends SubsystemBase implements DiagnosticSubsystem {

  private OdometryThread odometryThread;
  private Phoenix6Odometry phoenix6Odometry = new Phoenix6Odometry();
  private PositionEstimator positionEstimator;
  final private Phoenix6DrivebaseState state;

  private SwerveModule swerveModules[] = new SwerveModule[Constants.Drivebase.MODULES.length];

  private PIDController headingController = new PIDController(
      Constants.Drivebase.PIDs.HEADING_CONTROL_kP,
      Constants.Drivebase.PIDs.HEADING_CONTROL_kI,
      Constants.Drivebase.PIDs.HEADING_CONTROL_kD);

  private Pigeon2 gyro = new Pigeon2(Constants.Drivebase.PIGEON_ID, Constants.Drivebase.CANIVORE_NAME);
  private StructArrayPublisher<SwerveModuleState> desiredSwervestate = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Desired swervestate", SwerveModuleState.struct).publish();
  private StructArrayPublisher<SwerveModuleState> actualSwervestate = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Actual swervestate", SwerveModuleState.struct).publish();

  private Pose2d cachedEstimatedRobotPose = new Pose2d();

  public Drivebase() {
    // Creates a pheonix 6 pro state based on the gyro -- the only sensor owned
    // directly by the drivebase. A pheonix 6 pro state is a class to store all
    // of a subsystems pheonix 6 pro sensor inputs
    state = phoenix6Odometry.registerDrivebase(gyro);

    Translation2d[] moduleLocations = new Translation2d[Constants.Drivebase.MODULES.length];

    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i] = new SwerveModule(Constants.Drivebase.MODULES[i], phoenix6Odometry);
      moduleLocations[i] = swerveModules[i].moduleLocation;
    }

    // Constructs a pose estimator with the pheonix 6 pro state, the state of
    // the swerve modules, and a lock. A pheonix 6 pro state is a class
    // to store all of a subsystems pheonix 6 pro sensor inputs
    positionEstimator = new PositionEstimator(
        this.getState(),
        Arrays.stream(swerveModules)
            .map(module -> module.getState())
            .toArray(Phoenix6SwerveModuleState[]::new),
        phoenix6Odometry::setReadLock,
        moduleLocations);

    
    // Reset the heading of the pose estimator to the correct side of the field. 
    // This ensures that camera heading estimates and swerve drive pose estimator estimates 
    // are ~ the same, so the robot doesn't spiral off the field.
    positionEstimator.reset(
      (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) ? 
          new Pose2d(TeleopFeature.FIELD_CENTER, Rotation2d.k180deg) : 
          new Pose2d(TeleopFeature.FIELD_CENTER, new Rotation2d()));

    Pigeon2Configuration gyroConfiguration = new Pigeon2Configuration();
    gyroConfiguration.MountPose.MountPoseYaw = 0;
    gyro.getConfigurator().apply(gyroConfiguration);
    resetGyroHeading();

    odometryThread = new OdometryThread(phoenix6Odometry, positionEstimator);

    new Vision(
      positionEstimator::addVisionMeasurement,
      VisionConstants.Comp2025Mount.VISION_IO_CONSTANTS
    );

    headingController.enableContinuousInput(0, 360);
    odometryThread.startThread();
  }

  @Override
  public void periodic() {
    cacheEstimatedRobotPose();
    SmartDashboard.putNumber("Gyro Position", gyro.getYaw().getValueAsDouble());
  }

  /**
   * Mitigate the skew resulting from rotating and driving simaltaneously.
   * @param speeds The chassis speeds inputs. This function modifies those speeds.
   */
  private void mitigateSkew(ChassisSpeeds speeds) {
    double cX = speeds.vxMetersPerSecond;
    double cY = speeds.vyMetersPerSecond;
    double cRot = speeds.omegaRadiansPerSecond;
    double magnitudeSpeed = Math.sqrt(Math.pow(cX, 2) + Math.pow(cY, 2));
    double k = Constants.Drivebase.SKEW_PROPORTIONAL;

    speeds.vxMetersPerSecond = cX + k * cRot * Math.sin(-Math.atan2(cX, cY) + Math.PI / 2) * magnitudeSpeed;
    speeds.vyMetersPerSecond = cY - k * cRot * Math.cos(-Math.atan2(cX, cY) + Math.PI / 2) * magnitudeSpeed;
  }

  /**
   * This function calls the {@code setSwerveModuleStates} function based on
   * the a desired translation, a desired rotation, and isFieldReletave boolean.
   * This function should be excecuted once every tick for smooth movement.
   */
  public void drive(double xMetersPerSecond, double yMetersPerSecond,
      double degreesPerSecond, boolean isFieldRelative) {
    ChassisSpeeds chassisSpeeds;
    double radiansPerSecond = Units.degreesToRadians(degreesPerSecond);
    if (isFieldRelative) {
      state.getReadLock().lock();
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xMetersPerSecond,
          yMetersPerSecond,
          radiansPerSecond,
          state.getGyroAngle());
      state.getReadLock().unlock();
    } else {
      chassisSpeeds = new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, radiansPerSecond);
    }

    mitigateSkew(chassisSpeeds);
    positionEstimator.getReadLock().lock();
    SwerveModuleState[] swerveModuleStates = positionEstimator.swerveDriveKinematics
        .toSwerveModuleStates(chassisSpeeds);
    positionEstimator.getReadLock().unlock();
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates,
        Constants.Drivebase.Info.MAX_MODULE_SPEED);
    setModuleStates(swerveModuleStates);
  }

  private void setModuleStates(SwerveModuleState[] moduleStates) {
    desiredSwervestate.set(moduleStates);
    actualSwervestate.set(getSwerveModuleStates());
    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i].setSwerveModuleState(moduleStates[i]);
    }
  }

  public void resetGyroHeading() {
    gyro.reset();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      gyro.setYaw(180);
    }
  }

  public void setAllDriveMotorBreakMode(boolean breakMode) {
    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i].setBrakeMode(breakMode);
    }
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    positionEstimator.stateLock.readLock().lock();
    SwerveModuleState[] moduleStates = new SwerveModuleState[Constants.Drivebase.MODULES.length];
    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      moduleStates[i] = swerveModules[i].getState().getSwerveModuleState();
    }
    ChassisSpeeds chassisSpeeds = positionEstimator.swerveDriveKinematics.toChassisSpeeds(moduleStates);
    positionEstimator.stateLock.readLock().unlock();
    return chassisSpeeds;
  }

  private SwerveModuleState[] getSwerveModuleStates() {
    phoenix6Odometry.setReadLock(true);
    SwerveModuleState[] moduleStates = new SwerveModuleState[Constants.Drivebase.MODULES.length];
    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      moduleStates[i] = swerveModules[i].getState().getSwerveModuleState();
    }
    phoenix6Odometry.setReadLock(false);
    return moduleStates;
  }

  public void setAllModulesTurnPidActive() {
    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i].setTurnControllerActive(true);
    }
  }

  /** 
   * Cache the latest estimate of the robot's pose a local variable.
   * Should only be called in Drivebase periodic, which runs every loop. Doing this so that 
   * lock and unlock don't get called too frequently on the position estimator.
   */
  private void cacheEstimatedRobotPose() {
    positionEstimator.getReadLock().lock();
    cachedEstimatedRobotPose = positionEstimator.swerveDrivePoseEstimator.getEstimatedPosition();
    positionEstimator.getReadLock().unlock();
  }

  /**
   * 
   * @return 
   *  The latest field-relative robot pose estimated by the position estimator
   *  (a local variable of the drivebase, updated by {@code cacheEstimatedRobotPose} 
   *  once per loop in periodic)
   */
  public Pose2d getCachedEstimatedRobotPose() {
    return cachedEstimatedRobotPose;
  }

  @Override
  public Command getErrorCommand(
      ErrorGroup errorGroupHandler) {
    Command[] swerveModuleCommandArray = new Command[Constants.Drivebase.MODULES.length];
    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModuleCommandArray[i] = swerveModules[i].TestConnectionThenModule(errorGroupHandler);
    }
    return Commands.parallel(swerveModuleCommandArray);
  }

  public static Alliance getAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() : Alliance.Blue;
  }

  /**
   * @return The swerve drive command to be used in Teleop. Heading is corrected with PID.
   */
  public Command getSwerveCommand(
      DoubleSupplier getXMetersPerSecond,
      DoubleSupplier getYMetersPerSecond,
      DoubleSupplier getOmegaDegreesPerSecond,
      boolean fieldRelative) {

    // Workaround, this counts as effectively final
    Rotation2d[] lastRecordedHeading = {new Rotation2d()};

    return Commands.sequence(
        getBaseSwerveCommand(
            getXMetersPerSecond,
            getYMetersPerSecond,
            getOmegaDegreesPerSecond,
            fieldRelative).until(
                (BooleanSupplier) () -> Math.abs(getOmegaDegreesPerSecond.getAsDouble()) == 0.0)
            .beforeStarting(
              () -> {
                DataLogManager.log("Base Swerve Start");
              }
            ).finallyDo(
              () -> {
                DataLogManager.log("Base Swerve End");
              }
            ),
        getSwerveHeadingCorrected(
            getXMetersPerSecond,
            getYMetersPerSecond,
            (Supplier<Rotation2d>) () -> lastRecordedHeading[0],
            true).beforeStarting(
                () -> {
                  phoenix6Odometry.setReadLock(true);
                  lastRecordedHeading[0] = state.getGyroAngle();
                  phoenix6Odometry.setReadLock(false);
                }
            )
            .until(
                (BooleanSupplier) () -> Math.abs(getOmegaDegreesPerSecond.getAsDouble()) > 0.0)
        )
        .repeatedly();
  }

  /**
   * Intented to be used for targeting features exclusively.
   */
  public Command getSwerveHeadingCorrected(
      DoubleSupplier getXMetersPerSecond,
      DoubleSupplier getYMetersPerSecond,
      Supplier<Rotation2d> getDesiredHeading,
      boolean isFieldRelative) {

    return getBaseSwerveCommand(
        getXMetersPerSecond,
        getYMetersPerSecond,
        (DoubleSupplier) () -> {

          phoenix6Odometry.setReadLock(true);
          double rotSpeed = headingController.calculate(
            state.getGyroAngle().getDegrees(),
            getDesiredHeading.get().getDegrees()
          );
          phoenix6Odometry.setReadLock(false);

          return rotSpeed;
        },
        isFieldRelative).beforeStarting(
          () -> {
            DataLogManager.log("Heading Control Swerve Start");
          }
        ).finallyDo(
          () -> {
            DataLogManager.log("Heading Control Swerve End");
          }
        );
  }

  /**
   * A basic swerve drive command. Intended to be used exclusively within other
   * commands in drivebase.
   */
  private Command getBaseSwerveCommand(
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

    Command command = runEnd(
        () -> {
          drive(
              xMetersPerSecond.getAsDouble() * fieldOrientationMultiplier,
              yMetersPerSecond.getAsDouble() * fieldOrientationMultiplier,
              degreesPerSecond.getAsDouble(),
              isFieldRelative);
        },
        () -> {
        }
    ).beforeStarting(
            () -> {
              setAllModulesTurnPidActive();
            });

    command.addRequirements(this);

    return command;
  }

  public Phoenix6DrivebaseState getState() {
    return state;
  }
}