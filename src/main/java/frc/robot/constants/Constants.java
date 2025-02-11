// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

// TODO: add all robot constant values when they have been decided
public class Constants {

  public class Testing {
    // if ENSURE_COMPETITION_READY_SUBSYSTEMS is true, all subystems
    // must be constructed and assigned to the correct variable in Robot.java.
    // If some subsystems are not created and this value is true, an exeption
    // will be thrown.
    public static final boolean ENSURE_COMPETITION_READY_SUBSYSTEMS = true;

    public static final double NUMBER_OF_MOTOR_ROTATIONS_FOR_MODULE_TEST = 1.0;
    public static final double TURN_MOTOR_ROTATION_SPEED = 0.15;
    public static final double TURN_MOTOR_AND_ENCODER_TOLERANCE = 0.05;
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
    public static final String CANIVORE_NAME = "Practice Swerve";

    public static final double MAX_METERS_PER_SECOND = 0.0;
    public static final double MAX_DEGREES_PER_SECOND = 0.0;

      public class PIDs {
        public static final double SWERVE_MODULE_TURN_kP = 0.013;
        public static final double SWERVE_MODULE_TURN_kI = 0.0;
        public static final double SWERVE_MODULE_TURN_kD = 0.00025;
        public static final double SWERVE_MODULE_TURN_kF = 0.0;
        public static final double SWERVE_MODULE_DRIVE_kP = 0.125;
        public static final double SWERVE_MODULE_DRIVE_kI = 0.0;
        public static final double SWERVE_MODULE_DRIVE_kD = 0.0;
        public static final double SWERVE_MODULE_DRIVE_kF = 0.1075;

        public static final boolean SMART_PID_ENABLED = false;
        public static final boolean SMART_PID_TURN_ENABLED = true;
        public static final boolean SMART_PID_DRIVE_ENABLED = true;

      public static final double PID_LOW_LIMIT = -0.8;
      public static final double PID_HIGH_LIMIT = 0.8;
    }

    public static SwerveModuleConstants MODULES[] = {
        new SwerveModuleConstants(
            9, 3, 4, -0.212402, new Translation2d(0.28194, 0.28194), "Front Left"),
        new SwerveModuleConstants(
            11, 7, 8, 0.120361, new Translation2d(0.28194, -0.28194), "Front Right"),
        new SwerveModuleConstants(
            12, 1, 2, -0.377441, new Translation2d(-0.28194, 0.28194), "Back Left"),
        new SwerveModuleConstants(
            10, 5, 6, 0.096680, new Translation2d(-0.28194, -0.28194), "Back Right")
    };

    public class Info {
      public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
      public static final double WHEEL_DIAMETER = 0.0991108;
      public static final double REVS_PER_METER = DRIVE_MOTOR_GEAR_RATIO / (WHEEL_DIAMETER * Math.PI);
      public static final double TURN_MOTOR_GEAR_RATIO = 150.0 / 7.0;

      public static final double MAX_MODULE_SPEED = 4.498848;
    }
  }

  public class Elevator {
    public static final int MOTOR_ID = 0;

    // This tolerance value will be used for deciding if the elevator
    // should target to its setpoint or if the setpoint is too far
    // away and the elevator should just maintain its current position.
    public static final double TOLORENCE_METERS_FOR_SETPOINT = 0.0;
    // This tolerance value will be used for moving to a setpoint
    // using the MoveToPositionCommand.
    public static final double TOLERENCE_METERS_FOR_MOVE_TO_POSITION = 0.0;
    public static final double ROTATIONS_TO_METERS = 0.0;

    public class PIDs {
      public static final double ELEVATOR_kP = 0.0;
      public static final double ELEVATOR_kI = 0.0;
      public static final double ELEVATOR_kD = 0.0;
      public static final double ELEVATOR_kF = 0.0;
    }

    public class Profile {
      public static final double MAX_VELOCITY = 0.0;
      public static final double MAX_ACCELERATION = 0.0;
    }

    public class Setpoints {
      // All positions are in meters
      public static final double FLOOR_POSITION = 0.0;
      public static final double L1_POSITION = 0.0;
      public static final double L2_POSITION = 0.0;
      public static final double L3_POSITION = 0.0;
      public static final double L4_POSITION = 0.0;
      public static final double NET_POSITION = 0.0;
    }
  }

  public class ClimberIDs {
    public static final int CLIMBER_KRAKEN_MOTOR = 12;
    public static final int CLIMBER_MAGNET_SENSOR_1 = 0;
    public static final int CLIMBER_MAGNET_SENSOR_2 = 0;

    public static final double CLIMBER_KP = 0.1;
    public static final double CLIMBER_KD = 0.0;
    public static final double CLIMBER_KI = 0.0;
    public static final double CLIMBER_KF = 0.0;

    public static final boolean CLIMBER_SMARTPID_ACTIVE = false;

    public static final double CLIMBER_MAX = 2.0; // in motor rotations TODO figure out max rotations
    public static final double CLIMBER_MIN = -2.0; // in motor rotations

    public static final double CLIMBER_VELOCITY = 5; // TODO figure out velocity

    public static final double CLIMBER_RANGE = 0.1; // TODO figure out range
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
        public static final int TRANSLATION_JOYSTICK_ID = 2;
        public static final int BUTTON_STICK_ID = 0;
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
          public static final int COLLECT_CORAL = 11;
          public static final int SCORE_CORAL = 12;
        }
      }
    }
  }
}