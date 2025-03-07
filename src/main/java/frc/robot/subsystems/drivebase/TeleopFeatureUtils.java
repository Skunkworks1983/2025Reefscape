
package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Constants.Drivebase.TeleopFeature;

public class TeleopFeatureUtils {

  /**
   * @param target The field location to point at.
   * @return The heading needed for the robot to point at the target.
   */
  public static Rotation2d getTargetingAngle(Translation2d target, Pose2d robotPose) {
    Rotation2d angle = new Rotation2d(target.getX() - robotPose.getX(), target.getY() - robotPose.getY());
    return angle;
  }

  /** @return the angle needed to point at the nearest face of the reef */
  public static Rotation2d getPointAtReefFaceAngle(Pose2d robotPose) {
    double angleDegrees = getTargetingAngle(getReefCenter(), robotPose).getDegrees();

    // Rounding to the nearest 60 degrees (the reef is hexagontal)
    return Rotation2d.fromDegrees(Math.round(angleDegrees / 60.0) * 60.0);
  }

  /**
   * @return the heading to point at the coral station corresponding
   *  to the side of the field the robot is on.
   */
  public static Rotation2d getPointAtCoralStationAngle(Pose2d robotPose) {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      return getTargetingAngle(
        getReefCenter(),
        robotPose
      ).getDegrees() > 0.0 ? TeleopFeature.RED_LEFT_CORAL_STATION_ANGLE : TeleopFeature.RED_RIGHT_CORAL_STATION_ANGLE;

    } else {
      return getTargetingAngle(
        getReefCenter(),
        robotPose
      ).getDegrees() < 0.0 ? TeleopFeature.BLUE_LEFT_CORAL_STATION_ANGLE : TeleopFeature.BLUE_RIGHT_CORAL_STATION_ANGLE;
    }
  }

  public static Translation2d getReefCenter() {
    return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) ? 
      TeleopFeature.REEF_RED : TeleopFeature.REEF_BLUE;
  }

  public static double getReefFaceSpeedX(Rotation2d targetingAngle, double speed) {
    if(Math.abs(MathUtil.inputModulus(targetingAngle.getDegrees(), -180, 180)) > 90) {
      return Math.sin(targetingAngle.getRadians()) * speed;
    }
    else {
      return -Math.sin(targetingAngle.getRadians()) * speed;
    }
  }

  public static double getReefFaceSpeedY(Rotation2d targetingAngle, double speed) {
    if(Math.abs(MathUtil.inputModulus(targetingAngle.getDegrees(), -180, 180)) > 90) {
      return -Math.cos(targetingAngle.getRadians()) * speed;
    }
    else {
      return Math.cos(targetingAngle.getRadians()) * speed;
    }
  }

  public static Rotation2d getCoralCycleAngleNoOdometry(boolean isHoldingCoral, Rotation2d gyroHeading) {

    if(isHoldingCoral) {
      return Rotation2d.fromDegrees(Math.round(gyroHeading.getDegrees() / 60.0) * 60.0);
    } else {
      if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
          if(
            Math.abs(TeleopFeature.RED_LEFT_CORAL_STATION_ANGLE.minus(gyroHeading).getDegrees()) <
            Math.abs(TeleopFeature.RED_RIGHT_CORAL_STATION_ANGLE.minus(gyroHeading).getDegrees())
          ) {
            return TeleopFeature.RED_LEFT_CORAL_STATION_ANGLE;
          }
          else {
            return TeleopFeature.RED_RIGHT_CORAL_STATION_ANGLE;
          }
      } else {
        if(
            Math.abs(TeleopFeature.BLUE_LEFT_CORAL_STATION_ANGLE.minus(gyroHeading).getDegrees()) <
            Math.abs(TeleopFeature.BLUE_RIGHT_CORAL_STATION_ANGLE.minus(gyroHeading).getDegrees())
          ) {
            return TeleopFeature.BLUE_LEFT_CORAL_STATION_ANGLE;
          }
          else {
            return TeleopFeature.BLUE_RIGHT_CORAL_STATION_ANGLE;
          }
      }
    }
  }
}
