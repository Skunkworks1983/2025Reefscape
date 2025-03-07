// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

// TODO: add all robot constant values when they have been decided
public class Constants {

  public class CurrentLimits {

    // Measured in amps
    public static final double KRAKEN_CURRENT_LIMIT_VALUE = 90.0;
    public static final double MINI_KRAKEN_CURRENT_LIMIT_VALUE = 70.0;
    public static final int NEO_550_CURRENT_LIMIT_VALUE = 25; // not used

    public static final CurrentLimitsConfigs KRAKEN_CURRENT_LIMIT_CONFIG;
    public static final CurrentLimitsConfigs MINI_KRAKEN_CURRENT_LIMIT_CONFIG;

    static {
      KRAKEN_CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs();
      KRAKEN_CURRENT_LIMIT_CONFIG.StatorCurrentLimit 
        = KRAKEN_CURRENT_LIMIT_CONFIG.SupplyCurrentLimit
        = KRAKEN_CURRENT_LIMIT_VALUE;

      KRAKEN_CURRENT_LIMIT_CONFIG.StatorCurrentLimitEnable 
        = KRAKEN_CURRENT_LIMIT_CONFIG.SupplyCurrentLimitEnable
        = true;

      MINI_KRAKEN_CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs();
      MINI_KRAKEN_CURRENT_LIMIT_CONFIG.StatorCurrentLimit 
        = MINI_KRAKEN_CURRENT_LIMIT_CONFIG.SupplyCurrentLimit
        = MINI_KRAKEN_CURRENT_LIMIT_VALUE;

      MINI_KRAKEN_CURRENT_LIMIT_CONFIG.StatorCurrentLimitEnable 
        = MINI_KRAKEN_CURRENT_LIMIT_CONFIG.SupplyCurrentLimitEnable
        = true;
    };
  }

  public class Testing {
    // if ENSURE_COMPETITION_READY_SUBSYSTEMS is true, all subystems
    // must be constructed and assigned to the correct variable in Robot.java.
    // If some subsystems are not created and this value is true, an exeption
    // will be thrown.
    public static final boolean ENSURE_COMPETITION_READY_SUBSYSTEMS = true;

    public static enum Robot {
      Comp2024,
      Comp2025
    }
  
    // Change this to test on the 2024 robot's drivebase.
    public static Robot ROBOT = Robot.Comp2025;

    public static final double NUMBER_OF_MOTOR_ROTATIONS_FOR_MODULE_TEST = 1.0;
    public static final double TURN_MOTOR_ROTATION_SPEED = 0.15;
    public static final double TURN_MOTOR_AND_ENCODER_TOLERANCE = 0.05;

    public static final double CLIMBER_HEIGHT_CHANGE = 0.05;
    public static final double CLIMBER_CURRENT_TOLERANCE = 10; //TODO find tolerance
    public static final double ELEVATOR_MAX_SPEED = .08;
  }

  public class Collector {

    public class IDs {
      public static final int RIGHT_MOTOR = 42; //42 is the real id
      public static final int LEFT_MOTOR = 11;
      public static final int DIGITAL_INPUT_CHANNEL = 1;
    }

    public class Speeds {
      public static final double CORAL_INTAKE_SLOW_SPEED = 8.0; //meters per sec
      public static final double CORAL_INTAKE_FAST_SPEED = 18.0; //meters per sec 
      public static final double CORAL_EXPEL_SLOW_SPEED = -50.0; //meters per sec
      public static final double CORAL_EXPEL_FAST_SPEED = 18.0; //meters per sec 
      public static final double ALGAE_INTAKE_SPEED = 5.0; //meters per sec
      public static final double ALGAE_EXPEL_SPEED = -5.0; //meters per sec

      public static final double SPEED_MULIPILER_LEFT = 0.75;
    }

    public static final double COLLECTOR_ROTATIONS_PER_METER = 0.0762 * Math.PI;
    public static final double END_COUNT_TICK_COUNTER_ALGAE = 3;
    public static final double END_COUNT_TICK_COUNTER_CORAL = 2;
    public static final double COLLECTOR_AMPS_BEFORE_CUTTOF = 5.0;
    public static final double ALGAE_AMP_CUT_OFF = 6.0;

      public static final boolean SMART_PID_ENABLED = false;

  public class PositionControlMode {
      public static final double KP = 5.0;
      public static final double KI = 0.0;
      public static final double KD = 0.0;
      public static final double KF = 0.0;
      public static final double KV = 1.3;
      public static final double KA = 0.0;
      public static final double KS = 0.0;
    }

  public class VelocityControlMode {
      public static final double KP = 0.0;
      public static final double KI = 0.0;
      public static final double KD = 0.0;
      public static final double KF = 0.0;
      public static final double KV = 1.3;
      public static final double KA = 0.0;
      public static final double KS = 0.0;
    }
  }

  public class Drivebase {
    public static final String CANIVORE_NAME = Testing.ROBOT == Testing.Robot.Comp2025 ? "Drivebase 2025" : "1983 Comp Drivebase";
    public static final int PIGEON_ID = Testing.ROBOT == Testing.Robot.Comp2025 ? 22 : 26;
    public static final int LIDAR_RIGHT_DATA_PORT = 8;
    public static final int LIDAR_RIGHT_TRIGGER_PORT = 7;
    public static final int LIDAR_LEFT_DATA_PORT = 4;
    public static final int LIDAR_LEFT_TRIGGER_PORT = 3;
    public static final int LIDAR_TRIGGER_DISTANCE = 60;
    public static final double MAX_METERS_PER_SECOND = 4.5;
    public static final double MAX_DEGREES_PER_SECOND = 270;

    public class IDS {
      public static int ROTATION_JOYSTICK_ID = 1;
      public static int TRANSLATION_JOYSTICK_ID = 0;
      public static int BUTTON_STICK_ID = 2;
    }

    

    // All modules are at the position (+-MODULE_TO_OFFSET, +-MODULE_TO_OFFSET)
    public static double MODULE_OFFSET = 0.288925;
    
    // For 2024 Robot
    public static final double T_X = 0.925;
    public static final double T_Y = 0.8041666; 

    public static SwerveModuleConstants MODULES[] = (Testing.ROBOT == Testing.Robot.Comp2025) ? new SwerveModuleConstants[] {
      new SwerveModuleConstants(
        10, 11, 12, -0.337158 + .75, new Translation2d(-MODULE_OFFSET, MODULE_OFFSET), "Back Left"
      ),
      new SwerveModuleConstants(
        13, 14, 15, -0.289795 + .25, new Translation2d(-MODULE_OFFSET, -MODULE_OFFSET), "Back Right"
      ),
      new SwerveModuleConstants(
        16, 17, 18, 0.476318 - .75, new Translation2d(MODULE_OFFSET, MODULE_OFFSET), "Front Left"
      ),
      new SwerveModuleConstants(
        19, 20, 21, -0.353027 + .75, new Translation2d(MODULE_OFFSET, -MODULE_OFFSET), "Front Right"
      )
    } :
    new SwerveModuleConstants[] {
    	new SwerveModuleConstants(18, 16, 17, 0.311035, new Translation2d(T_X, T_Y),
    		"Front Left"),
    	new SwerveModuleConstants(12, 10, 11, -0.415283, new Translation2d(T_X,
    		-T_Y), "Front Right"),
    	new SwerveModuleConstants(23, 25, 24, -0.205566, new Translation2d(-T_X,
    		T_Y), "Back Left"),
    	new SwerveModuleConstants(20, 22, 21, 0.308838, new Translation2d(-T_X,
    		-T_Y), "Back Right")
    };

    public class Info {
      public static final double DRIVE_MOTOR_GEAR_RATIO = Testing.ROBOT == Testing.Robot.Comp2025 ? 6.12 : 6.75;
      public static final double WHEEL_DIAMETER = 0.0991108;
      public static final double REVS_PER_METER = DRIVE_MOTOR_GEAR_RATIO / (WHEEL_DIAMETER * Math.PI);
      public static final double METERS_PER_REV = 1.0 / REVS_PER_METER;
      public static final double TURN_MOTOR_GEAR_RATIO = 150.0 / 7.0;

      public static final double MAX_MODULE_SPEED = 4.498848;
    }

    public class PIDs {
      public static final double SWERVE_MODULE_TURN_KP = 0.0145;
      public static final double SWERVE_MODULE_TURN_KI = 0.0;
      public static final double SWERVE_MODULE_TURN_KD = 0.00017;
      public static final double SWERVE_MODULE_TURN_KF = 0.0;
      public static final double SWERVE_MODULE_DRIVE_KP = 0.125;
      public static final double SWERVE_MODULE_DRIVE_KI = 0.0;
      public static final double SWERVE_MODULE_DRIVE_KD = 0.0;
      public static final double SWERVE_MODULE_DRIVE_KF = 0.0;
      public static final double SWERVE_MODULE_DRIVE_KV = 0.1075;
      public static final double SWERVE_MODULE_DRIVE_KA = 0.0;
      public static final double SWERVE_MODULE_DRIVE_KS = 0.0;

      public static final double HEADING_CONTROL_kP = 6.00;
      public static final double HEADING_CONTROL_kI = 0.0;
      public static final double HEADING_CONTROL_kD = 0.0;
      
      public static final double PID_LOW_LIMIT = -0.8;
      public static final double PID_HIGH_LIMIT = 0.8;

      public static final boolean SMART_PID_ENABLED = false;
      public static final boolean SMART_PID_TURN_ENABLED = false;
      public static final boolean SMART_PID_DRIVE_ENABLED = false;
    }

    public class TeleopFeature {
      public static final Translation2d FIELD_CENTER = new Translation2d(FIELD_X_LENGTH / 2.0, FIELD_Y_LENGTH / 2.0);
      public static final Translation2d APPROXIMATE_BLUE_START_POSE = FIELD_CENTER.plus(new Translation2d(0.0, -1.5));
      public static final Translation2d APPROXIMATE_RED_START_POSE = FIELD_CENTER.plus(new Translation2d(0.0, 1.5));
      public static final Translation2d REEF_BLUE = new Translation2d(4.0259, 4.48945);
      public static final Translation2d REEF_RED = new Translation2d(FIELD_X_LENGTH-4.0259, 4.48945);
      public static final Rotation2d BLUE_LEFT_CORAL_STATION_ANGLE = Rotation2d.fromDegrees(-45);
      public static final Rotation2d BLUE_RIGHT_CORAL_STATION_ANGLE = Rotation2d.fromDegrees(45);
      public static final Rotation2d RED_LEFT_CORAL_STATION_ANGLE = Rotation2d.fromDegrees(135);
      public static final Rotation2d RED_RIGHT_CORAL_STATION_ANGLE = Rotation2d.fromDegrees(-135);
      public static final double REEF_SMALL_RADIUS = Units.inchesToMeters(93.50 / 2);
      public static final double BRANCH_LINEUP_HORIZONTAL_OFFSET = 1.0;
      public static final double BRANCH_LINEUP_DIST_FROM_REEF = 1.0;
      public static final double MAX_DIST_FROM_REEF_CENTER = 3.0;
    }

    public static final double FIELD_X_LENGTH = 17.55; // Meters
    public static final double FIELD_Y_LENGTH = 8.05; // Meters
    public static final double SKEW_PROPORTIONAL = .027;
    public static final double CURRENT_LIMIT = 100.0;
  }

  public class Funnel {
    public class IDs {
      public static final int PIVOT_MOTOR_ID = 23;
    }

    public static final double FUNNEL_TOLERANCE = 0.1;

    public class PIDs {
      public static final double FUNNEL_KP = .08;
      public static final double FUNNEL_KD = 0.0;
      public static final double FUNNEL_KI = 0.0;
      public static final double FUNNEL_KF = 0.0;

      public static final boolean FUNNEL_SMARTPID_ACTIVE = false;
    }

    public static final double PIVOT_MOTOR_GEAR_RATIO = 1.0 / 100.0;
    public static final double FUNNEL_POSITION_LOW_CONVERTED = 0.0; //TODO FIGURE OUT POSITIONS IN ROTATIONS
    public static final double FUNNEL_POSITION_HIGH_IN_DEGREES = 40.0;
    public static final double FUNNEL_POSITION_HIGH_CONVERTED = (FUNNEL_POSITION_HIGH_IN_DEGREES / 360) / PIVOT_MOTOR_GEAR_RATIO;
  }

  public class Elevator {
    // For determining right and left, look at the elevator from the side paralel to the one that the elevator is on
    public static final int MOTOR_RIGHT_ID = 27;
    public static final int MOTOR_LEFT_ID = 28;
    public static final int BOTTOM_LIMIT_SWITCH_ID = 5;
    public static final int TOP_LIMIT_SWITCH_ID = 6;

    // This tolerance value will be used for deciding if the elevator
    // should target to its setpoint or if the setpoint is too far
    // away and the elevator should just maintain its current position.
    public static final double TOLORENCE_METERS_FOR_SETPOINT = 0.0;
    // This tolerance value will be used for moving to a setpoint
    // using the MoveToPositionCommand.
    public static final double TOLERENCE_METERS_FOR_MOVE_TO_POSITION = 0.25;

    // In meters
    public static final double MAX_HEIGHT_CARRIAGE = 1.527175;
    public static final double MAX_HEIGHT_STAGE_ONE = 0.7366;
    public static final double STAGE_ONE_TO_CARRIAGE_HEIGHT = MAX_HEIGHT_CARRIAGE / MAX_HEIGHT_STAGE_ONE;
    public static final double GEAR_RATIO = 1.0/6.25;
    public static final double ROTATIONS_TO_METERS = 0.1016 * STAGE_ONE_TO_CARRIAGE_HEIGHT;
    public static final double MOTOR_ROTATIONS_TO_METERS = 1;
    public static final double METERS_TO_MOTOR_ROTATIONS = 1; // We need to fix Transforms after Competition


    public class PIDs {
      public static final double ELEVATOR_kP = 1.5;
      public static final double ELEVATOR_kI = 0.125;
      public static final double ELEVATOR_kD = 0.0;
      public static final double ELEVATOR_kF = 0.0;
      public static final double ELEVATOR_kV = 0.0;
      public static final double ELEVATOR_kA = 0.0;
      public static final double ELEVATOR_kS = 0.55;

      public static final boolean SMART_PID_ENABLED = false;
    }

    public class Profile {
      public static final double MAX_VELOCITY = 30;
      public static final double MAX_ACCELERATION = 65;
    }
  }

  public class Wrist {
    public class IDs {
      public static final int WRIST_KRAKEN_MOTOR_ID = 4;
      public static final int WRIST_TOP_MAGNET_SENSOR = 0; // Magnet Sensor ids are currently not working
      public static final int WRIST_BOTTOM_MAGNET_SENSOR = 2;

    }
    
    public class PIDs {
      public static final double WRIST_KA = 0.0;
      public static final double WRIST_KS = 0.0;
      public static final double WRIST_KV = 0.0;
      public static final double WRIST_KP = 3.0;
      public static final double WRIST_KD = 0.0;
      public static final double WRIST_KI = 0.0;
      public static final double WRIST_KF = 0.0;

      public static final boolean WRIST_SMARTPID_ACTIVE = false;
    }

    public static final double WRIST_MAX_VELOCITY = 1;
    public static final double WRIST_MAX_ACCELERATION = 2; 

    public static final double WRIST_TOLERANCE = 0.02;

    public static final int WRIST_GEAR_RATIO = 56; // 56 motor rotations to 1 wrist rotation

    public static final double WRIST_MIDPOINT_ROTATIONS = 0.2 * WRIST_GEAR_RATIO; // TODO: figure out postitions
    public static final double WRIST_MIN_ROTATIONS = 0;
    public static final double WRIST_MAX_ROTATIONS = 0.4 * WRIST_GEAR_RATIO;

  }
  
  // TODO: add end effector setpoints
  public class EndEffectorSetpoints {

    public static final double WRIST_STOW_POSITION_CORAL = 0.0;
    public static final double WRIST_STOW_POSITION_ALGAE = 0.1;

    public static final EndEffectorSetpointConstants ALGAE_GROUND = 
      new EndEffectorSetpointConstants(0.0, 0.3941, WRIST_STOW_POSITION_ALGAE);
    public static final EndEffectorSetpointConstants ALGAE_STOW = 
      new EndEffectorSetpointConstants(0.0, WRIST_STOW_POSITION_ALGAE, WRIST_STOW_POSITION_ALGAE);
    public static final EndEffectorSetpointConstants ALGAE_PROCESSOR = 
      new EndEffectorSetpointConstants(0.0, 0.0, WRIST_STOW_POSITION_ALGAE);
    public static final EndEffectorSetpointConstants ALGAE_L2 = 
      new EndEffectorSetpointConstants(0.0, 0.0, WRIST_STOW_POSITION_ALGAE);
    public static final EndEffectorSetpointConstants ALGAE_L3 = 
      new EndEffectorSetpointConstants(0.0, 0.0, WRIST_STOW_POSITION_ALGAE);
    public static final EndEffectorSetpointConstants ALGAE_NET = 
      new EndEffectorSetpointConstants(0.0, 0.0, WRIST_STOW_POSITION_ALGAE);

    public static final EndEffectorSetpointConstants CORAL_GROUND = 
      new EndEffectorSetpointConstants(0.0, 0.2, WRIST_STOW_POSITION_CORAL);
    public static final EndEffectorSetpointConstants CORAL_STOW = 
      new EndEffectorSetpointConstants(0.0, WRIST_STOW_POSITION_CORAL, WRIST_STOW_POSITION_CORAL);
    public static final EndEffectorSetpointConstants CORAL_L1 = 
      new EndEffectorSetpointConstants(0.0, 0.0, WRIST_STOW_POSITION_CORAL);
    public static final EndEffectorSetpointConstants CORAL_L2 = 
      new EndEffectorSetpointConstants(13.756, 0.1441, WRIST_STOW_POSITION_CORAL);
    public static final EndEffectorSetpointConstants CORAL_L3 = 
      new EndEffectorSetpointConstants(24.914, 0.0, WRIST_STOW_POSITION_CORAL);
    public static final EndEffectorSetpointConstants CORAL_L4 = 
      new EndEffectorSetpointConstants(39.0, 0.0, WRIST_STOW_POSITION_CORAL);
  };

  public class Climber {
    public class IDs {
      public static final int CLIMBER_KRAKEN_MOTOR = 12;
      public static final int CLIMBER_MAGNET_SENSOR_1 = 3;
      public static final int CLIMBER_MAGNET_SENSOR_2 = 4;
    }

    public class PIDs {
      public static final double CLIMBER_KP = 0.1; //TODO tune constants
      public static final double CLIMBER_KD = 0.0;
      public static final double CLIMBER_KI = 0.0;
      public static final double CLIMBER_KF = 0.0;
    }

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

      // TODO: add button IDs
      public class Buttons {

        // Switch being off corresponds to coral
        // Switch being on corresponds to algae
        public static final int ALGAE_TOGGLE = 10;

        // The following buttons depend on ALGAE_TOGGLE
        public static final int GOTO_SCORE_LOW = 15; // either L1 or proccesor on ALGAE_TOGGLE
        public static final int GOTO_L2 = 13; // either place coral L2 or remove algae L2
        public static final int GOTO_L3 = 12; // either place coral L3 or remove algae L4
        public static final int GOTO_SCORE_HIGH = 11; // either L4 or net depending on ALGAE_TOGGLE
        public static final int GOTO_STOW = 23; 
        public static final int GOTO_GROUND = 22;

        // The effects of these buttons may change depending on algae or coral mode.
        // Will also change for different positions (e.g. net)
        public static final int INTAKE = 17;
        public static final int EXPEL = 24;

        public static final int CLIMBER_GOTO_MAX = 0;
        public static final int CLIMBER_GOTO_MIN = 0;

        public static final int TARGET_REEF_BUTTON = 1;
        public static final int TARGET_CORAL_STATION_BUTTON = 2;
        public static final int TARGET_CORAL_CYCLE_NO_ODOMETRY_BUTTON = 3;

        public static final int RAISE_FUNNEL_TOGGLE = 0;

        public static final int FUNNEL_GO_TO_MAX = 0;
        public static final int FUNNEL_GO_TO_MIN = 0;
      }
    }
  }

  public class Phoenix6Odometry {
    public static final double updatesPerSecond = 100.0;
  }
}
