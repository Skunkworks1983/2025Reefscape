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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.utils.error.ErrorGroup;
import frc.robot.utils.error.DiagnosticSubsystem;

public class Drivebase extends SubsystemBase implements DiagnosticSubsystem {

  private SwerveModule swerveModules[] = new SwerveModule[Constants.Drivebase.MODULES.length];
  // TODO private Pigeon2 gyro = new Pigeon2(26, Constants.Drivebase.CANIVORE_NAME);
  private StructArrayPublisher<SwerveModuleState> desiredSwervestate = NetworkTableInstance.getDefault().getStructArrayTopic("Desired swervestate", SwerveModuleState.struct).publish();
  private StructArrayPublisher<SwerveModuleState> actualSwervestate = NetworkTableInstance.getDefault().getStructArrayTopic("Actual swervestate", SwerveModuleState.struct).publish();


  private SwerveDriveKinematics swerveDriveKinematics;
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private final Field2d swerveOdometryField2d = new Field2d();

  public Drivebase() {
    Translation2d[] moduleLocations = new Translation2d[Constants.Drivebase.MODULES.length];

    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
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
      new Pose2d()
    );

    SmartDashboard.putData("Swerve Drive Odometry", swerveOdometryField2d);
    swerveOdometryField2d.setRobotPose(new Pose2d());
    
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
   * Must be called every loop in <code>periodic()</code> to keep odometry up to
   * date.
   */
  public void updateOdometry() {
    swerveDrivePoseEstimator.update(
      getGyroAngle(),
      getSwerveModulePositions()
    );

    swerveOdometryField2d.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
  }

  /** Reset the <code>SwerveDrivePoseEstimator</code> to the given pose. */
  public void resetOdometry(Pose2d newPose) {
    swerveDrivePoseEstimator.resetPosition(
      getGyroAngle(),
      Arrays.stream(swerveModules)
        .map(swerveModule -> swerveModule.getSwerveModulePosition())
        .toArray(SwerveModulePosition[]::new),
      newPose
    );
  }

  // TODO: add docstring
  private void drive(double xMetersPerSecond, double yMetersPerSecond,
    double degreesPerSecond, boolean isFieldRelative
  ) {
    ChassisSpeeds chassisSpeeds;
    double radiansPerSecond = Units.degreesToRadians(degreesPerSecond);
    if (isFieldRelative) {
      chassisSpeeds = 
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xMetersPerSecond, 
            yMetersPerSecond,
            radiansPerSecond, 
            getGyroAngle()
          );
    } else {
      chassisSpeeds = new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, radiansPerSecond);
    }

    SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
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
      swerveModules[i].setSwerveModulState(moduleStates[i]);
    }
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[Constants.Drivebase.MODULES.length];
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
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
    double angleDegrees = 0.0; // TODO -gyro.getYaw().getValueAsDouble();
    SmartDashboard.putNumber("Gyro", angleDegrees);
    return Rotation2d.fromDegrees(angleDegrees);
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {

    SwerveModuleState[] moduleStates = new SwerveModuleState[Constants.Drivebase.MODULES.length];
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      moduleStates[i] = swerveModules[i].getSwerveModuleState();
    }

    return swerveDriveKinematics.toChassisSpeeds(moduleStates);
  }

  private SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[Constants.Drivebase.MODULES.length];
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      moduleStates[i] = swerveModules[i].getSwerveModuleState();
    }
    return moduleStates;
  }

  public void setAllModulesTurnPidActive() {
    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i].setTurnControllerActive(true);
    }
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(),
        getGyroAngle());
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
