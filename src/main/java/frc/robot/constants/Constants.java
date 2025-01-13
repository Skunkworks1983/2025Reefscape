// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class Constants {

    public class Drivebase {
    
        public class PIDs {
            public static float TURN_kP; 
            public static float TURN_kI; 
            public static float TURN_kD; 
            public static float TURN_kF; 

            public static float DRIVE_kP; 
            public static float DRIVE_kI; 
            public static float DRIVE_kD; 
            public static float DRIVE_kF; 
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

        public class IDS {
            public static int LEFT_JOYSTICK_ID = 1;
            public static int RIGHT_JOYSTICK_ID = 2;
            public static int BUTTON_STICK_ID = 3;
        }
    }

}
