// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

// TODO: add all robot constant values when they have been decided
public class Constants {

    public class Wrist {
        public class PIDs {

            public static final double WRIST_MOTOR_kP = 0.0;
            public static final double WRIST_MOTOR_kI = 0.0;
            public static final double WRIST_MOTOR_kD = 0.0;
            public static final double WRIST_MOTOR_kF = 0.0;
        }

        public static final double TOLORENCE_DEGREES_FOR_SETPOINT = 0.0;
        // This tolerance value is for moving to a setpoint
        // using the MoveToPositionCommand.
        public static final double TOLERENCE_DEGREES_FOR_MOVE_TO_POSITION = 0.0;
        public static final double WRIST_REVS_TO_DEGREES = 180 / Math.PI;

        public class WristProfile {
            public static final double MAX_VELOCITY = 0.0;
            public static final double MAX_ACCELERATION = 0.0;
        }

        public class WristSetpoints {
            public static final double FLOOR_POSITION_DEGREES = 0.0;
            public static final double SCORING_POSITION_DEGREES = 0.0;
            public static final double STOWED_POSITION_DEGREES = 0.0;
        }
    }

    public class Drivebase {

        public static final String CANIVORE_NAME = "Practice Swerve";

        public class PIDs {
            public static double SWERVE_MODULE_TURN_kP = 1.0;
            public static double SWERVE_MODULE_TURN_kI = 1.0;
            public static double SWERVE_MODULE_TURN_kD = 1.0;
            public static double SWERVE_MODULE_TURN_kF = 1.0;
            public static double SWERVE_MODULE_DRIVE_kP = 1.0;
            public static double SWERVE_MODULE_DRIVE_kI = 1.0;
            public static double SWERVE_MODULE_DRIVE_kD = 1.0;
            public static double SWERVE_MODULE_DRIVE_kF = 1.0;

            public static final boolean SMART_PID_ENABLED = true;
            public static final boolean SMART_PID_TURN_ENABLED = true;
            public static final boolean SMART_PID_DRIVE_ENABLED = true;

            public static final double PID_LOW_LIMIT = -0.8;
            public static final double PID_HIGH_LIMIT = 0.8;

        }

        public static SwerveModuleConstants MODULES[] = {
                new SwerveModuleConstants(
                        9, 3, 4, -0.212402, new Translation2d(0.28194, 0.28194), "Front Left"),
                new SwerveModuleConstants(
                        11, 7, 8, 0.120361, new Translation2d(0.28194, -0.28194), "Front Right"

                ),
                new SwerveModuleConstants(
                        12, 1, 2, -0.377441, new Translation2d(-0.28194, 0.28194), "Back Left"),
                new SwerveModuleConstants(
                        10, 5, 6, 0.096680, new Translation2d(-0.28194, -0.28194), "Back Right")
        };

        public class Info {
            public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
            public static final double WHEEL_DIAMETER = 0.0991108;
            public static final double REVS_PER_METER = DRIVE_MOTOR_GEAR_RATIO / (WHEEL_DIAMETER * Math.PI);

            public static final double MAX_MODULE_SPEED = 4.498848;
        }
    }

    public class OI {
        public class LIMITS {
            public static double MAX_INSTRUCTED_METERS_PER_SECOND = 2.0;
            public static double MAX_INSTRUCTED_DEGREES_PER_SECOND = 90.0;
        }

        public static double AXIS_DEADBAND = .08;

        // (x or y joystick axis input after deadband ^ AXIS_INPUT_EXPONENT)
        // * MAX_INSTRUCTED_METERS_PER_SECOND = instructed meters per second
        // Ensure that this AXIS_INPUT_EXPONENT does not result in a result
        // that is always positive.
        public static double AXIS_INPUT_EXPONENT = 3;

        public class IDS {
            public static int ROTATION_JOYSTICK_ID = 1;
            public static int TRANSLATION_JOYSTICK_ID = 2;
            public static int BUTTON_STICK_ID = 3;
        }
    }

}
