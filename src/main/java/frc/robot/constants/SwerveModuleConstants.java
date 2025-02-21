package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModuleConstants {
    public int turnMotorId;
    public int driveMotorId;
    public int turnEncoderId;
    public double turnEncoderOffset;
    public Translation2d moduleLocation;
    public String moduleName;
    public SwerveModuleConstants(
      int turnMotorId, 
      int driveMotorId, 
      int turnEncoderId, 
      double turnEncoderOffset, 
      Translation2d moduleLocation,
      String moduleName
    ) {
        this.turnMotorId = turnMotorId;
        this.driveMotorId = driveMotorId;
        this.turnEncoderId = turnEncoderId;
        this.turnEncoderOffset = turnEncoderOffset;
        this.moduleLocation = moduleLocation;
        this.moduleName = moduleName;
    }
}

