// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.parser.ParseException;
import edu.wpi.first.wpilibj2.command.Command;

// TODO: add all robot constant values when they have been decided
public class Constants {

  public class PathPlanner
  {
    public static final double PATHPLANNER_DRIVE_KP = 1.0;
    public static final double PATHPLANNER_DRIVE_KD = .0;
    public static final double PATHPLANNER_DRIVE_KI = .0;
    public static final double PATHPLANNER_DRIVE_KF = .0;

    public static final double PATHPLANNER_TURN_KP = 1.0;
    public static final double PATHPLANNER_TURN_KD = .0;
    public static final double PATHPLANNER_TURN_KI = .0;
    public static final double PATHPLANNER_TURN_KF = .0;

    public static final double ROBOT_LENGTH = 0.864; //in meters with bumpers
    public static final double ROBOT_WIDTH = 0.864; // in meters with bumpers

    public static final double ROBOT_MASS = 67.1317; //kilograms
    public static final double MOMENT_OF_INERTIA = 1.0/12.0 * (ROBOT_MASS)*(Math.sqrt(ROBOT_WIDTH) + Math.sqrt(ROBOT_LENGTH));
  
    
    public static final double PATHPLANNER_MAX_METERS_PER_SECOND = .0; //TODO give real number

    // distance from center to wheel
    public static final double PATHPLANNER_DRIVEBASE_RADIUS_METERS = 0.; //TODO give real number

    public static final double UPDATE_PERIOD = .02; //seconds
  }

  public class Testing {
    // if ENSURE_COMPETITION_READY_SUBSYSTEMS is true, all subystems
    // must be constructed and assigned to the correct variable in Robot.java.
    // If some subsystems are not created and this value is true, an exeption
    // will be thrown.
    public static final boolean ENSURE_COMPETITION_READY_SUBSYSTEMS = true;

    public static final double NUMBER_OF_MOTOR_ROTATIONS_FOR_MODULE_TEST = 1.0;
    public static final double TURN_MOTOR_ROTATION_SPEED = 0.15;
    public static final double TURN_MOTOR_AND_ENCODER_TOLERANCE = 0.05;

    public static final double CLIMBER_HEIGHT_CHANGE = 0.05;
    public static final double CLIMBER_CURRENT_TOLERANCE = 10; //TODO find tolerance
  }

  public class Collector {
    public static final int RIGHT_MOTOR = 42;
    public static final int LEFT_MOTOR = 11;

    public static final double COLLECTOR_ROTATIONS_PER_METER = 0.0762 * Math.PI;

    public static final double COLLECOR_ROTATE_SLOW = 4.0;
    public static final double COLLECOR_ROTATE_FAST = 6.0;

    public static final double COLLECTOR_AMPS_BEFORE_CUTTOF = 3.0;
    public static final double SECONDS_BEFORE_CUTTOF = 0.5;

    public class PIDs {
      public static final double KP = 0.0;
      public static final double KI = 0.0;
      public static final double KD = 0.0;
      public static final double KF = 1.3;
      public static final boolean SMART_PID_ENABLED = true;

    }
  }

  public class Drivebase {
    public static final String CANIVORE_NAME = "Evil Canivore";
    public static final int PIGEON_ID = 22;
    public static final double MAX_METERS_PER_SECOND = 4.5;
    public static final double MAX_DEGREES_PER_SECOND = 270;

    public class IDS {
      public static int ROTATION_JOYSTICK_ID = 1;
      public static int TRANSLATION_JOYSTICK_ID = 0;
      public static int BUTTON_STICK_ID = 3;
    }

    // All modules are at the position (+-MODULE_TO_OFFSET, +-MODULE_TO_OFFSET)
    public static double MODULE_OFFSET = 0.288925;

    public static SwerveModuleConstants MODULES[] = {
      new SwerveModuleConstants(
        16, 17, 18, 0.476318 - .75, new Translation2d(MODULE_OFFSET, MODULE_OFFSET), "Front Left"
      ),
      new SwerveModuleConstants(
        19, 20, 21, -0.353027 + .75, new Translation2d(MODULE_OFFSET, -MODULE_OFFSET), "Front Right"
      ),
      new SwerveModuleConstants(
        10, 11, 12, -0.337158 + .75, new Translation2d(-MODULE_OFFSET, MODULE_OFFSET), "Back Left"
      ),
      new SwerveModuleConstants(
        13, 14, 15, -0.289795 + .25, new Translation2d(-MODULE_OFFSET, -MODULE_OFFSET), "Back Right"
      )
    };

    public class Info {
      public static final double DRIVE_MOTOR_GEAR_RATIO = 6.12;
      public static final double WHEEL_DIAMETER = 0.0991108;
      public static final double REVS_PER_METER = DRIVE_MOTOR_GEAR_RATIO / (WHEEL_DIAMETER * Math.PI);
      public static final double METERS_PER_REV = 1.0 / REVS_PER_METER;
      public static final double TURN_MOTOR_GEAR_RATIO = 150.0 / 7.0;

      public static final double MAX_MODULE_SPEED = 4.498848;
    }

    public class PIDs {
      public static final double SWERVE_MODULE_TURN_kP = 0.0145;
      public static final double SWERVE_MODULE_TURN_kI = 0.0;
      public static final double SWERVE_MODULE_TURN_kD = 0.00017;
      public static final double SWERVE_MODULE_TURN_kF = 0.0;
      public static final double SWERVE_MODULE_DRIVE_kP = 0.125;
      public static final double SWERVE_MODULE_DRIVE_kI = 0.0;
      public static final double SWERVE_MODULE_DRIVE_kD = 0.0;
      public static final double SWERVE_MODULE_DRIVE_kF = 0.1075;
      public static final double HEADING_CONTROL_kP = 3.00;
      public static final double HEADING_CONTROL_kI = 0.0;
      public static final double HEADING_CONTROL_kD = 0.0;
      public static final double PID_LOW_LIMIT = -0.8;
      public static final double PID_HIGH_LIMIT = 0.8;

      public static final boolean SMART_PID_ENABLED = true;
      public static final boolean SMART_PID_TURN_ENABLED = true;
      public static final boolean SMART_PID_DRIVE_ENABLED = true;
    }

    public class FieldTarget {
      public static final Translation2d REEF_BLUE = new Translation2d(4.0259, 4.48945);
      public static final Translation2d REEF_RED = new Translation2d(FIELD_X_LENGTH-4.0259, 4.48945);
    }

    public static final double FIELD_X_LENGTH = 17.55; // Meters
    public static final double FIELD_Y_LENGTH = 8.05; // Meters
    public static final double SKEW_PROPORTIONAL = .027;
  }

  public class VisionConstants {

    public static final String FRONT_CAMERA_NAME = "Camera_0";
    public static final String SIDE_CAMERA_NAME = "Camera_1";

    private static final Transform3d MOUNT_TO_FRONT_CAMERA = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(1.351),
            Units.inchesToMeters(-1.268),
            Units.inchesToMeters(-0.81)),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(-19.27),
            Units.degreesToRadians(-15.0)));

    private static final Transform3d MOUNT_TO_SIDE_CAMERA = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-1.050),
            Units.inchesToMeters(1.365078),
            Units.inchesToMeters(-0.762394)),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(-27.225),
            Units.degreesToRadians(97.0)));

    private static final Transform3d ROBOT_TO_MOUNT =
      new Transform3d(
        new Translation3d( // TODO: check these transformation estimations
          .305,
          .305,
          Units.inchesToMeters(8.25)
        ),
        new Rotation3d(
            0.0,
            0.0,
            0.0));

    public static final Transform3d ROBOT_TO_FRONT_CAMERA = ROBOT_TO_MOUNT.plus(MOUNT_TO_FRONT_CAMERA);
    public static final Transform3d ROBOT_TO_SIDE_CAMERA = ROBOT_TO_MOUNT.plus(MOUNT_TO_SIDE_CAMERA);

    public static final double MAX_AMBIGUITY = 0.3;
    public static final double LINEAR_STD_DEV_BASELINE = 0.02;
    public static final double ANGULAR_STD_DEV_BASELINE = 0.06;
    public static final double MAX_Z_ERROR = 3.0;
    public static final double MAX_AVERAGE_TAG_DISTANCE = 3.0; // Meters
  }

  public class Funnel {
    public static final int PIVOT_MOTOR_ID = 4;
    public static final double PIVOT_MOTOR_GEAR_RATIO = 1 / 100;

    public static final double FUNNEL_KP = 1; //TODO tune constants
    public static final double FUNNEL_KD = 0.0;
    public static final double FUNNEL_KI = 0.0;
    public static final double FUNNEL_KF = 0.0;

    public static final boolean FUNNEL_SMARTPID_ACTIVE = false;

    public static final double FUNNEL_POSITION_1 = 0.0; //TODO FIGURE OUT POSITIONS IN ROTATIONS
    public static final double FUNNEL_POSITION_2 = 0.0;
    public static final double FUNNEL_POSITION_3 = 0.0;
    }

  public class Elevator {
    public static final int MOTOR_ID = 12;
    public static final int BOTTOM_LIMIT_SWITCH_ID = 4;
    public static final int TOP_LIMIT_SWITCH_ID = 5;

    // This tolerance value will be used for deciding if the elevator
    // should target to its setpoint or if the setpoint is too far
    // away and the elevator should just maintain its current position.
    public static final double TOLORENCE_METERS_FOR_SETPOINT = 0.0;
    // This tolerance value will be used for moving to a setpoint
    // using the MoveToPositionCommand.
    public static final double TOLERENCE_METERS_FOR_MOVE_TO_POSITION = 0.0;
    // In meters
    public static final double MAX_HEIGHT_CARRIAGE = 1.527175;
    public static final double MAX_HEIGHT_STAGE_ONE = 0.7366;
    public static final double STAGE_ONE_TO_CARRIAGE_HEIGHT = MAX_HEIGHT_CARRIAGE / MAX_HEIGHT_STAGE_ONE;
    public static final double GEAR_RATIO = 1.0/5.0;
    public static final double ROTATIONS_TO_METERS = 0.1016 * STAGE_ONE_TO_CARRIAGE_HEIGHT;
    public static final double MOTOR_ROTATIONS_TO_METERS = GEAR_RATIO * ROTATIONS_TO_METERS;
    public static final double METERS_TO_MOTOR_ROTATIONS = 1 / MOTOR_ROTATIONS_TO_METERS;


    public class PIDs {
      public static final double ELEVATOR_kP = 1.25;
      public static final double ELEVATOR_kI = 0.0;
      public static final double ELEVATOR_kD = 0.15;
      public static final double ELEVATOR_kV = 0.0;
      public static final double ELEVATOR_kS = 0.0;
      public static final boolean SMART_PID_ENABLED = false;
    }

    public class Profile {
      public static final double MAX_VELOCITY = 60.0;
      public static final double MAX_ACCELERATION = 80.0;
    }

    public class Setpoints {
      // All positions are in meters
      public static final double FLOOR_POSITION = 0.0;
      public static final double L1_POSITION = 0.0;
      public static final double L2_POSITION = 0.0;
      public static final double L3_POSITION = 0.0;
      public static final double L4_POSITION = MAX_HEIGHT_CARRIAGE;
      public static final double NET_POSITION = 0.0;
    }
  }

  public class WristIDs {
    public static final int WRIST_KRAKEN_MOTOR_ID = 12; // ID 12 is for the test board
    public static final int WRIST_MAGNET_SENSOR_1 = 0;
    
    public static final double WRIST_KS = 0.0;
    public static final double WRIST_KV = 0.12;
    public static final double WRIST_KP = 5.0;
    public static final double WRIST_KD = 0.1;
    public static final double WRIST_KI = 0.0;
    public static final double WRIST_KF = 0.0;

    public static final boolean WRIST_SMARTPID_ACTIVE = false;

    public static final double WRIST_VELOCITY = 1; 

    public static final double WRIST_RANGE = 0.03;

    public static final double WRIST_MIDPOINT_ROTATIONS = 2.5; //TODO figure out postitions
    public static final double WRIST_MIN_ROTATIONS = -5;
    public static final double WRIST_MAX_ROTATIONS = 0;
  }

  public class ClimberIDs {
    public static final int CLIMBER_KRAKEN_MOTOR = 12;
    public static final int CLIMBER_MAGNET_SENSOR_1 = 4;
    public static final int CLIMBER_MAGNET_SENSOR_2 = 5;

    public static final double CLIMBER_KP = 0.1; //TODO tune constants
    public static final double CLIMBER_KD = 0.0;
    public static final double CLIMBER_KI = 0.0;
    public static final double CLIMBER_KF = 0.0;

    public static final boolean CLIMBER_SMARTPID_ACTIVE = false;

    public static final double CLIMBER_MAX = Units.inchesToMeters(12); // in meters TODO figure out max height
    public static final double CLIMBER_MIN = 0.0; // in meters

    public static final double CLIMBER_TOLERANCE = 0.001;

    public static final double CLIMBER_GEAR_RATIO = 1.0 / 20.0; //TODO check with vince (he said 20 to 1, i think i did the math right but idk)
    public static final double CLIMBER_ROTATIONS_TO_METERS = Units.inchesToMeters(0.25);
    public static final double CLIMBER_MOTOR_ROTATIONS_TO_CLIMBER_HEIGHT = CLIMBER_GEAR_RATIO * CLIMBER_ROTATIONS_TO_METERS;
  }

  public class OI {
    public class LIMITS {
      public static final double MAX_INSTRUCTED_METERS_PER_SECOND = Constants.Drivebase.MAX_METERS_PER_SECOND;
      public static final double MAX_INSTRUCTED_DEGREES_PER_SECOND = Constants.Drivebase.MAX_DEGREES_PER_SECOND;
    }

    public static final double AXIS_DEADBAND = .08;

    // (x or y joystick axis input after deadband ^ AXIS_INPUT_EXPONENT)
    // * MAX_INSTRUCTED_METERS_PER_SECOND = instructed meters per second
    // Ensure that this AXIS_INPUT_EXPONENT does not result in a result
    // that is always positive.
    public static final double AXIS_INPUT_EXPONENT = 3.0;

    public class IDs {
      public class Joysticks {
        public static final int ROTATION_JOYSTICK_ID = 1;
        public static final int TRANSLATION_JOYSTICK_ID = 0;
        public static final int BUTTON_STICK_ID = 2;
      }

      public class Buttons {
        public class Elevator {
          public static final int GOTO_FLOOR_POSITION = 0;
          public static final int GOTO_L1 = 0;
          public static final int GOTO_L2 = 0;
          public static final int GOTO_L3 = 0;
          public static final int GOTO_L4 = 0;
          public static final int GOTO_NET = 0;
        }

        public class Collector {
          public static final int ROTATE_CORAL = 23;
          public static final int INTAKE_CORAL = 14;
          public static final int COLLECT_CORAL = 11;
          public static final int SCORE_CORAL = 12;
        }

        public class Wrist {
          public static final int WRIST_UP = 18;
          public static final int WRIST_DOWN = 13;
        }

        public class Drivebase {
          public static final int TARGET_REEF_BUTTON = 1;
        }

        public class Climber{
          public static final int GO_TO_MAX = 10;

        }
      }
    }
  }

  public class Phoenix6Odometry {
    public static final double updatesPerSecond = 100.0;
  }
}