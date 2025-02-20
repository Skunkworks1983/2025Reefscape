// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
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

import org.json.simple.parser.ParseException;

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
    // Creates a pheonix 6 pro state based on the gyro -- the only sensor owned 
    // directly by the drivebase. A pheonix 6 pro state is a class to store all
    // of a subsystems pheonix 6 pro sensor inputs
    state = phoenix6Odometry.registerDrivebase(gyro);

    Translation2d[] moduleLocations = new Translation2d[Constants.Drivebase.MODULES.length];

    for(int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
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
      moduleLocations
    );

    Pigeon2Configuration gyroConfiguration = new Pigeon2Configuration();
    gyroConfiguration.MountPose.MountPoseYaw = 0; 
    gyro.getConfigurator().apply(gyroConfiguration);
    resetGyroHeading();

    odometryThread = new OdometryThread(phoenix6Odometry, positionEstimator);

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

    odometryThread.startThread();

    //configurePathPlanner();
    RobotConfig config = new RobotConfig(
      Constants.PathPlanner.ROBOT_MASS, 
      Constants.PathPlanner.MOI, 
      new ModuleConfig(
        Constants.Drivebase.Info.WHEEL_DIAMETER / 2, 
        Constants.Drivebase.Info.MAX_MODULE_SPEED, 
        1, 
        DCMotor.getKrakenX60(1).withReduction(Constants.Drivebase.Info.DRIVE_MOTOR_GEAR_RATIO), 
        100, 
        4
        ),
      Constants.PathPlanner.MODULE_OFFSETS);
      positionEstimator.stateLock.readLock().lock();
    AutoBuilder.configure(
      positionEstimator::getPose,positionEstimator::reset,
      this::getRobotRelativeSpeeds, (speeds, feedforwards) -> driveRobotRelative(speeds), 
      new PPHolonomicDriveController( 
        new PIDConstants(
          Constants.PathPlanner.PATHPLANNER_DRIVE_KP, 
          Constants.PathPlanner.PATHPLANNER_DRIVE_KI, 
          Constants.PathPlanner.PATHPLANNER_DRIVE_KD),
        new PIDConstants(
          Constants.PathPlanner.PATHPLANNER_TURN_KP, 
          Constants.PathPlanner.PATHPLANNER_TURN_KI, 
          Constants.PathPlanner.PATHPLANNER_TURN_KD),
          Constants.PathPlanner.PERIOD
      ),
      config,
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this
    );
    positionEstimator.stateLock.readLock().unlock();
  }

  @Override
  public void periodic() {}
  /**
   * This function calls the <code>setModuleStates</code> function based on
   * the a desired translation, a desired rotation, and isFieldReletave boolean.
   * This function should be excecuted once every tick for smooth movement.
   */
  private void drive(double xMetersPerSecond, double yMetersPerSecond,
    double degreesPerSecond, boolean isFieldRelative
  ) {
    ChassisSpeeds chassisSpeeds;
    double radiansPerSecond = Units.degreesToRadians(degreesPerSecond);
    if (isFieldRelative) {
      state.getReadLock().lock();
      chassisSpeeds = 
        ChassisSpeeds.fromFieldRelativeSpeeds(
          xMetersPerSecond, 
          yMetersPerSecond,
          radiansPerSecond, 
          state.getGyroAngle()
        );
    state.getReadLock().unlock();
    } else {
      chassisSpeeds = new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, radiansPerSecond);
    }

    driveRobotRelative(chassisSpeeds);
  }

  private void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
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
    gyro.reset();
  }

  public void setAllDriveMotorBreakMode(boolean breakMode) {
    for (int i = 0; i < Constants.Drivebase.MODULES.length; i++) {
      swerveModules[i].setBrakeMode(breakMode);
    }
  }

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

    public Command followPathCommand(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return AutoBuilder.followPath(path);
    }
    catch (ParseException p){
      System.out.println("pathplanner threw parseexception while parsing " + pathName);
    }
    catch (IOException e){
      System.out.println("pathplanner threw ioexception while parsing " + pathName);
    }
    return new Command(){};
   
  }

  public Command followAutoTrajectory(String autoName) {
    return new PathPlannerAuto(autoName);
  }
}
