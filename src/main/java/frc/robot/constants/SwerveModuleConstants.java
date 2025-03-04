package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModuleConstants {
    public final int turnMotorId;
    public final int driveMotorId;
    public final int turnEncoderId;
    public final double turnEncoderOffset;
    public final Translation2d moduleLocation;
    public final String moduleName;
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

