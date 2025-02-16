// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.utils.error.ErrorGroup;
import frc.robot.utils.error.DiagnosticSubsystem;

public class Drivebase extends SubsystemBase implements DiagnosticSubsystem {

  private SwerveModule swerveModules[] = new SwerveModule[Constants.Drivebase.MODULES.length];

  private PIDController headingController = new PIDController(
      Constants.Drivebase.PIDs.HEADING_CONTROL_kP,
      Constants.Drivebase.PIDs.HEADING_CONTROL_kI,
      Constants.Drivebase.PIDs.HEADING_CONTROL_kD);

  private AHRS gyro = new AHRS(NavXComType.kUSB1);
  // TODO private Pigeon2 gyro = new Pigeon2(26,
  // Constants.Drivebase.CANIVORE_NAME);
  private StructArrayPublisher<SwerveModuleState> desiredSwervestate = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Desired swervestate", SwerveModuleState.struct).publish();
  private StructArrayPublisher<SwerveModuleState> actualSwervestate = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Actual swervestate", SwerveModuleState.struct).publish();

  private SwerveDriveKinematics swerveDriveKinematics;
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private final Field2d swerveOdometryField2d = new Field2d();

  public Drivebase() {
    Translation2d[] moduleLocations = new Translation2d[Constants.Drivebase.MODULES.length];

    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i] = new SwerveModule(Constants.Drivebase.MODULES[i]);
      moduleLocations[i] = swerveModules[i].moduleLocation;
    }

    Pigeon2Configuration gConfiguration = new Pigeon2Configuration();
    gConfiguration.MountPose.MountPoseYaw = 0;
    // TODO gyro.getConfigurator().apply(gConfiguration);
    // TODO resetGyroHeading();

    swerveDriveKinematics = new SwerveDriveKinematics(moduleLocations);

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
        swerveDriveKinematics,
        getGyroAngle(),
        getSwerveModulePositions(),
        new Pose2d());

    SmartDashboard.putData("Swerve Drive Odometry", swerveOdometryField2d);
    swerveOdometryField2d.setRobotPose(new Pose2d());

    // Ensure robot code won't crash if the vision subsystem fails to initialize.
    try {
      new Vision(
          this::addVisionMeasurement,
          new VisionIOPhotonVision(
              VisionConstants.FRONT_CAMERA_NAME,
              VisionConstants.ROBOT_TO_FRONT_CAMERA),
          new VisionIOPhotonVision(
              VisionConstants.SIDE_CAMERA_NAME,
              VisionConstants.ROBOT_TO_SIDE_CAMERA));
    } catch (Exception exception) {
      System.out.println("Vision subsystem failed to initialize. See the below stacktrace for more details: ");
      exception.printStackTrace();
    }
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
   * Must be called every loop in periodic() to keep odometry up to
   * date.
   */
  public void updateOdometry() {
    swerveDrivePoseEstimator.update(
        getGyroAngle(),
        getSwerveModulePositions());

    swerveOdometryField2d.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
  }

  /** Reset the SwerveDrivePoseEstimator to the given pose. */
  public void resetOdometry(Pose2d newPose) {
    swerveDrivePoseEstimator.resetPosition(
        getGyroAngle(),
        Arrays.stream(swerveModules)
            .map(swerveModule -> swerveModule.getSwerveModulePosition())
            .toArray(SwerveModulePosition[]::new),
        newPose);
  }

  /**
   * @return the estimated pose of the robot from the
   *         {@link SwerveDrivePoseEstimator}'s
   *         odometry
   */
  public Pose2d getEstimatedRobotPose() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  // TODO: add docstring
  private void drive(double xMetersPerSecond, double yMetersPerSecond,
      double degreesPerSecond, boolean isFieldRelative) {
    ChassisSpeeds chassisSpeeds;
    double radiansPerSecond = Units.degreesToRadians(degreesPerSecond);
    if (isFieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xMetersPerSecond,
          yMetersPerSecond,
          radiansPerSecond,
          getGyroAngle());
    } else {
      chassisSpeeds = new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, radiansPerSecond);
    }

    SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates,
        Constants.Drivebase.Info.MAX_MODULE_SPEED);
    setModuleStates(swerveModuleStates);
  }

  private void setModuleStates(SwerveModuleState[] moduleStates) {
    desiredSwervestate.set(moduleStates);
    actualSwervestate.set(getSwerveModuleStates());
    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i].setSwerveModulState(moduleStates[i]);
    }
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[Constants.Drivebase.MODULES.length];
    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModulePositions[i] = swerveModules[i].getSwerveModulePosition();
    }
    return swerveModulePositions;
  }

  public void resetGyroHeading() {
    // TODO gyro.setYaw(0);
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

  private ChassisSpeeds getRobotRelativeSpeeds() {

    SwerveModuleState[] moduleStates = new SwerveModuleState[Constants.Drivebase.MODULES.length];
    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      moduleStates[i] = swerveModules[i].getSwerveModuleState();
    }

    return swerveDriveKinematics.toChassisSpeeds(moduleStates);
  }

  private SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[Constants.Drivebase.MODULES.length];
    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      moduleStates[i] = swerveModules[i].getSwerveModuleState();
    }
    return moduleStates;
  }

  public void setAllModulesTurnPidActive() {
    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i].setTurnControllerActive(true);
    }
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(),
        getGyroAngle());
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

  /**
   * Bound in OI.
   * 
   * @param target the field location to point at.
   * @return the heading needed for the robot to point at the target.
   */
  public Rotation2d getTargetingAngle(Translation2d target) {
    Pose2d robotPose = getEstimatedRobotPose();
    Rotation2d angle = new Rotation2d(target.getX() - robotPose.getX(), target.getY() - robotPose.getY());
    return angle;
  }

  /**
   * @return The swerve drive command to be used in Teleop. Heading is corrected
   *         with
   *         PID.
   */
  public Command getSwerveCommand(
      DoubleSupplier getXMetersPerSecond,
      DoubleSupplier getYMetersPerSecond,
      DoubleSupplier getOmegaDegreesPerSecond,
      boolean fieldRelative) {

    return Commands.either(
        getBaseSwerveCommand(getXMetersPerSecond, getYMetersPerSecond, getOmegaDegreesPerSecond, fieldRelative),
        getSwerveWaitCommand(getXMetersPerSecond, getYMetersPerSecond, this::getGyroAngle, fieldRelative),
        (BooleanSupplier) () -> (getOmegaDegreesPerSecond.getAsDouble() > 0.0));
  }

  /**
   * Intented to be used for position targeting exclusively.
   */
  public Command getSwerveHeadingCorrected(
      DoubleSupplier getXMetersPerSecond,
      DoubleSupplier getYMetersPerSecond,
      Supplier<Rotation2d> getDesiredHeading,
      boolean isFieldRelative) {

    return getBaseSwerveCommand(
        getXMetersPerSecond,
        getYMetersPerSecond,
        (DoubleSupplier) () -> headingController.calculate(
            getGyroAngle().getDegrees(),
            getDesiredHeading.get().getDegrees()),
        isFieldRelative);
  }

  private Command getSwerveWaitCommand(
      DoubleSupplier getXMetersPerSecond,
      DoubleSupplier getYMetersPerSecond,
      Supplier<Rotation2d> getHeading,
      boolean fieldRelative) {

    Rotation2d[] lastRecordedHeading = {};

    return Commands.sequence(
        new WaitCommand(Constants.Drivebase.SECONDS_UNTIL_HEADING_CONTROL),
        getSwerveHeadingCorrected(
            getXMetersPerSecond,
            getYMetersPerSecond,
            (Supplier<Rotation2d>) () -> lastRecordedHeading[0],
            true).beforeStarting(
                () -> lastRecordedHeading[0] = getHeading.get()));
  }

  /**
   * A basic swerve drive command. Intended to be used exclusively within other
   * commands in drivebase.
   * {@link #getSwerveCommand()}
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
    return runEnd(
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
        }).beforeStarting(
            () -> {
              setAllModulesTurnPidActive();
            });
  }
}
