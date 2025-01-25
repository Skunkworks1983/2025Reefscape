// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

// TODO: add all robot constant values when they have been decided
public class Constants {

  public class Drivebase {
  
    public class PIDs {
        public static final double SWERVE_MODULE_TURN_kP = 1.0;
        public static final double SWERVE_MODULE_TURN_kI = 1.0;
        public static final double SWERVE_MODULE_TURN_kD = 1.0;
        public static final double SWERVE_MODULE_TURN_kF = 1.0;
        public static final double SWERVE_MODULE_DRIVE_kP = 1.0;
        public static final double SWERVE_MODULE_DRIVE_kI = 1.0;
        public static final double SWERVE_MODULE_DRIVE_kD = 1.0;
        public static final double SWERVE_MODULE_DRIVE_kF = 1.0;
    }

    public static final SwerveModuleConstants MODULES[] = {
        new SwerveModuleConstants(
            0, 0
        ),
        new SwerveModuleConstants(
            0, 0

        ),
        new SwerveModuleConstants(
            0, 0
        ),
        new SwerveModuleConstants(
            0, 0
        )
    };
  }

  public class Elevator {
    public static final int MOTOR_ID = 0;

    // In order to stop moving based on profile and start
    // pathing to the overall position target.
    public static final double TOLORENCE_METERS = 0.0;
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
      public static final double FLOOR_POSITION_METERS = 0.0;
      public static final double L1_POSITION_METERS = 0.0;
      public static final double L2_POSITION_METERS = 0.0;
      public static final double L3_POSITION_METERS = 0.0;
      public static final double L4_POSITION_METERS = 0.0;
      public static final double NET_POSITION_METERS = 0.0;
    }
  }

  public class OI {
    public class LIMITS {
      public static final double MAX_INSTRUCTED_METERS_PER_SECOND = 0.0;
      public static final double MAX_INSTRUCTED_DEGREES_PER_SECOND = 0.0;
    }

    public static final double AXIS_DEADBAND = .08;

    // (x or y joystick axis input after deadband ^ AXIS_INPUT_EXPONENT) 
    //  * MAX_INSTRUCTED_METERS_PER_SECOND = instructed meters per second
    // Ensure that this AXIS_INPUT_EXPONENT does not result in a result
    // that is always positive.
    public static final double AXIS_INPUT_EXPONENT = 3.0;

    public class IDs {
      public class Joysticks {
        public static final int ROTATION_JOYSTICK_ID = 1;
        public static final int TRANSLATION_JOYSTICK_ID = 2;
        public static final int BUTTON_STICK_ID = 3;
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
      }
    }
  }
}
