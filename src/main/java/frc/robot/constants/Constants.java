// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

// TODO: add all constant values when they have been decided
public class Constants {

    public class Drivebase {
    
        public class PIDs {
            public static double SWERVE_MODULE_TURN_kP = 1.0;
            public static double SWERVE_MODULE_TURN_kI = 1.0;
            public static double SWERVE_MODULE_TURN_kD = 1.0;
            public static double SWERVE_MODULE_TURN_kF = 1.0;
            public static double SWERVE_MODULE_DRIVE_kP = 1.0;
            public static double SWERVE_MODULE_DRIVE_kI = 1.0;
            public static double SWERVE_MODULE_DRIVE_kD = 1.0;
            public static double SWERVE_MODULE_DRIVE_kF = 1.0;
        }

        public static class SwerveModuleConstants {
            public int turnMotorId;
            public int driveMotorId;
            public SwerveModuleConstants(int turnMotorId, int driveMotorId) {
                this.turnMotorId = turnMotorId;
                this.driveMotorId = driveMotorId;
            }
        }

        public static SwerveModuleConstants MODULES[] = {
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

    public class OI {
        public class LIMITS {
            public static double MAX_INSTRUCTED_METERS_PER_SECOND = 2.0;
            public static double MAX_INSTRUCTED_DEGREES_PER_SECOND = 90.0;
        }

        public static double AXIS_DEADBAND = .1;

        // (x or y joystick axis input after deadband ^ AXIS_INPUT_EXPONENT) 
        //  * MAX_INSTRUCTED_METERS_PER_SECOND = instructed meters per second
        public static double AXIS_INPUT_EXPONENT = 3;

        public class IDS {
            public static int LEFT_JOYSTICK_ID = 1;
            public static int RIGHT_JOYSTICK_ID = 2;
            public static int BUTTON_STICK_ID = 3;
        }
    }

}
