// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
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
import frc.robot.utils.DualLidar;
import frc.robot.utils.error.DiagnosticSubsystem;

import org.json.simple.parser.ParseException;

public class Drivebase extends SubsystemBase implements DiagnosticSubsystem {

  private OdometryThread odometryThread;
  private Phoenix6Odometry phoenix6Odometry = new Phoenix6Odometry();
  private PositionEstimator positionEstimator;
  final private Phoenix6DrivebaseState state;

  private SwerveModule swerveModules[] = new SwerveModule[Constants.Drivebase.MODULES.length];

  private PIDController headingController = new PIDController(
      Constants.Drivebase.PIDs.HEADING_CONTROL_kP,
      Constants.Drivebase.PIDs.HEADING_CONTROL_kI,
      Constants.Drivebase.PIDs.HEADING_CONTROL_kD
    );

  private Pigeon2 gyro = new Pigeon2(Constants.Drivebase.PIGEON_ID, Constants.Drivebase.CANIVORE_NAME);

  private DualLidar dualLidar = new DualLidar();

  private StructArrayPublisher<SwerveModuleState> desiredSwervestate = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Desired swervestate", SwerveModuleState.struct).publish();
  private StructArrayPublisher<SwerveModuleState> actualSwervestate = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Actual swervestate", SwerveModuleState.struct).publish();

  private Pose2d cachedEstimatedRobotPose = new Pose2d();
  private Rotation2d cachedGyroHeading = new Rotation2d();

  //DEBUG
  public int[] redCount = {0};
  public int[] blueCount = {0};
  public int[] unknownCount = {0};

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
        moduleLocations,
        this
      );

    
    // Reset the heading of the pose estimator to the correct side of the field. 
    // This ensures that camera heading estimates and swerve drive pose estimator estimates 
    // are ~ the same, so the robot doesn't spiral off the field.

    Pigeon2Configuration gyroConfiguration = new Pigeon2Configuration();

    gyroConfiguration.MountPose.MountPoseYaw = 0;
    gyro.getConfigurator().apply(gyroConfiguration);

    // Only put this code back in when NOT running an auto.
    // positionEstimator.reset(
    //  (DriverStation.getAlliance().isPresent()
    //   && DriverStation.getAlliance().get() == Alliance.Red) ? 
    //       new Pose2d(TeleopFeature.FIELD_CENTER, Rotation2d.k180deg) : 
    //       new Pose2d(TeleopFeature.FIELD_CENTER, new Rotation2d()));

    // resetGyroHeading();

    odometryThread = new OdometryThread(phoenix6Odometry, positionEstimator);

    new Vision(
      positionEstimator::addVisionMeasurement,
      VisionConstants.Comp2025Mount.IO_CONSTANTS
    );

    headingController.enableContinuousInput(0, 360);
    odometryThread.startThread();

    RobotConfig config = new RobotConfig(
      Constants.PathPlanner.ROBOT_MASS, 
      Constants.PathPlanner.MOMENT_OF_INERTIA, 
      new ModuleConfig(
        Constants.Drivebase.Info.WHEEL_DIAMETER / 2, 
        Constants.Drivebase.Info.MAX_MODULE_SPEED, 
        1, 
        DCMotor.getKrakenX60(1).withReduction(Constants.Drivebase.Info.DRIVE_MOTOR_GEAR_RATIO), 
        Constants.Drivebase.DRIVE_CURRENT_LIMIT,
        Constants.Drivebase.MODULES.length
      ),
      Constants.Drivebase.pathPlannerOrderedModules
    );

    

    // Optional<Alliance> alliance; 
    // while (!(alliance = DriverStation.getAlliance()).isPresent()) {
      
    // }
    // final boolean shouldFlip = alliance.get() == DriverStation.Alliance.Red;

    positionEstimator.stateLock.readLock().lock();
    AutoBuilder.configure(
      positionEstimator::getPose,
      positionEstimator::pathplannerReset,
      this::getRobotRelativeSpeeds,
      (speeds, feedforwards) -> driveRobotRelative(speeds), 
      new PPHolonomicDriveController( 
        new PIDConstants(
          Constants.PathPlanner.PATHPLANNER_DRIVE_KP, 
          Constants.PathPlanner.PATHPLANNER_DRIVE_KI, 
          Constants.PathPlanner.PATHPLANNER_DRIVE_KD),
        new PIDConstants(
          Constants.PathPlanner.PATHPLANNER_TURN_KP, 
          Constants.PathPlanner.PATHPLANNER_TURN_KI, 
          Constants.PathPlanner.PATHPLANNER_TURN_KD),
          Constants.RoboRIOInfo.UPDATE_PERIOD
      ),
      config,
//      () -> shouldFlip,
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE


           Optional<Alliance> alliance = DriverStation.getAlliance();

          if(alliance.isPresent()){ 
            if(alliance.get() == DriverStation.Alliance.Red){ 
              redCount[0]++;
            }
            else {
              blueCount[0]++;
            }
          } else{
            unknownCount[0]++;
          }
           return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
      },
      this
    );
    positionEstimator.stateLock.readLock().unlock();
    setAllModulesTurnPidActive();

    SmartDashboard.putBoolean("Auto Aligning", false); // Putting to SmartDashboard so you can pull it up before command actually starts
  }

  @Override
  public void periodic() {
    cacheEstimatedRobotPose();
    cacheGyroHeading();
    SmartDashboard.putNumber("Gyro Position", gyro.getYaw().getValueAsDouble());
    SmartDashboard.putBoolean("Lidar Right", dualLidar.isLidarRightTripped.getAsBoolean());
    SmartDashboard.putBoolean("Lidar Left", dualLidar.isLidarLeftTripped.getAsBoolean());
    SmartDashboard.putNumber("Lidar Right Distance", dualLidar.getLidarRightOutput());
    SmartDashboard.putNumber("Lidar Left Distance", dualLidar.getLidarLeftOutput());
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
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xMetersPerSecond,
        yMetersPerSecond,
        radiansPerSecond,
        getCachedGyroHeading()
      );
    } else {
      chassisSpeeds = new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, radiansPerSecond);
    }

    driveRobotRelative(chassisSpeeds);
  }

  private void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
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
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      resetGyroHeading(Rotation2d.fromDegrees(180));
    }
    else {
      resetGyroHeading(Rotation2d.fromDegrees(0));
    }
  }

  public void resetGyroHeading(Rotation2d newHeading) {
    positionEstimator.stateLock.writeLock().lock();
    gyro.reset();
    gyro.setYaw(newHeading.getDegrees());
    positionEstimator.stateLock.writeLock().unlock();
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

  public double calculateWithHeadingController(double targetHeading) {
    return headingController.calculate(
      getCachedGyroHeading().getDegrees(),
      targetHeading
    );
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

  private void cacheGyroHeading() {
    phoenix6Odometry.setReadLock(true);
    cachedGyroHeading = state.getGyroAngle();
    phoenix6Odometry.setReadLock(false);
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

  public Rotation2d getCachedGyroHeading() {
    return cachedGyroHeading;
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
                  lastRecordedHeading[0] = getCachedGyroHeading();
                }
            )
            .until(
                (BooleanSupplier) () -> Math.abs(getOmegaDegreesPerSecond.getAsDouble()) > 0.0)
        )
        .repeatedly();
  }
  public DualLidar getDualLidar(){
    return dualLidar;
  }

  public boolean AREWEREALLYGOINGRIGHT(boolean goingRight, Rotation2d targetHeading){
    if(goingRight == TeleopFeatureUtils.isCloseSideOfReef(targetHeading)) {
      return true;
    }
    else {
      return false;
    }
  }

  public Command getSwerveAlignCoral(
      DoubleSupplier getXMetersPerSecond,
      DoubleSupplier getYMetersPerSecond,
      boolean goingRight,
      double alignSpeed
    ) {

    double newAlignSpeed = alignSpeed * (goingRight ? -1 : 1);
    Rotation2d[] targetHeading = new Rotation2d[1];

    return Commands.sequence(
      getSwerveHeadingCorrected(
              () -> {return (getXMetersPerSecond.getAsDouble() * 0.5) + TeleopFeatureUtils.getReefFaceSpeedX(targetHeading[0], newAlignSpeed);},
              () -> {return (getYMetersPerSecond.getAsDouble() * 0.5) + TeleopFeatureUtils.getReefFaceSpeedY(targetHeading[0], newAlignSpeed);},
              () -> targetHeading[0],
              true
      ).beforeStarting(
        () -> {
          SmartDashboard.putBoolean("Auto Aligning", true);
          targetHeading[0] = TeleopFeatureUtils.getCoralCycleAngleNoOdometry(true, cachedGyroHeading);
          System.out.println("Target Heading: " + targetHeading[0] + " X: " + TeleopFeatureUtils.getReefFaceSpeedX(targetHeading[0], newAlignSpeed) + " Y: " + TeleopFeatureUtils.getReefFaceSpeedY(targetHeading[0], newAlignSpeed));
        }
      ).until(
        () -> {
          if(goingRight == TeleopFeatureUtils.isCloseSideOfReef(targetHeading[0])) {
            return dualLidar.isLidarRightTripped.getAsBoolean();
          }
          else {
            return dualLidar.isLidarLeftTripped.getAsBoolean();
          }
        }
      ),
      getSwerveHeadingCorrected(
              () -> {return TeleopFeatureUtils.getReefFaceSpeedX(targetHeading[0], -newAlignSpeed * 0.5);},
              () -> {return TeleopFeatureUtils.getReefFaceSpeedY(targetHeading[0], -newAlignSpeed * 0.5);},
              () -> targetHeading[0],
              true
      ).withTimeout(Constants.Drivebase.AUTO_ALIGN_MOVE_BACK_DURATION),
      getBaseSwerveCommand(
        () -> 0, 
        () -> 0, 
        () -> 0, 
        true
      ).withTimeout(0.04)
    ).raceWith(
      Commands.runEnd(
        () -> {
          System.out.println("Lidar left: " + dualLidar.getLidarLeftOutput() + " Lidar right: " + dualLidar.getLidarRightOutput() + " battery voltage: " + RobotController.getBatteryVoltage());
        },
        () -> {
          SmartDashboard.putBoolean("Auto Aligning", false);
        }
      )
    );
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

          double rotSpeed = calculateWithHeadingController(getDesiredHeading.get().getDegrees());

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
    int[] fieldOrientationMultiplier = new int[1];
    fieldOrientationMultiplier[0] = 1;

    Command command = runEnd(
        () -> {
          drive(
              xMetersPerSecond.getAsDouble() * fieldOrientationMultiplier[0],
              yMetersPerSecond.getAsDouble() * fieldOrientationMultiplier[0],
              degreesPerSecond.getAsDouble(),
              isFieldRelative);
        },
        () -> {
        }
    ).beforeStarting(
            () -> {
              setAllModulesTurnPidActive();
              Optional<Alliance> alliance = DriverStation.getAlliance();
              if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
                fieldOrientationMultiplier[0] = 1;
              } else {
                fieldOrientationMultiplier[0] = -1;
              }
            });

    command.addRequirements(this);

    return command;
  }

  public Phoenix6DrivebaseState getState() {
    return state;
  }

  public Command followPathCommand(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return AutoBuilder.followPath(path);
    }
    catch (ParseException p) {
      System.out.println("pathplanner threw parseexception while parsing " + pathName);
    }
    catch (IOException e) {
      System.out.println("pathplanner threw ioexception while parsing " + pathName);
    }
    return new Command() {};
   
  }


  public Command followAutoTrajectory(String autoName) {
    return new PathPlannerAuto(autoName);
  }

  public Command resetGyro() {
    return Commands.startEnd(
      () -> {
        resetGyroHeading();
      },
      () -> {

      }
    );
  }

}