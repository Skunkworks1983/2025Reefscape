
package frc.robot.subsystems.drivebase;

import java.util.function.Supplier;
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
  public static Rotation2d getTargetingAngle(Translation2d target, Supplier<Pose2d> getRobotPose) {
    Pose2d robotPose = getRobotPose.get();
    Rotation2d angle = new Rotation2d(target.getX() - robotPose.getX(), target.getY() - robotPose.getY());
    return angle;
  }

  /** @return the angle needed to point at the nearest face of the reef */
  public static Rotation2d getPointAtReefFaceAngle(Supplier<Pose2d> getRobotPose) {
    double angleDegrees = getTargetingAngle(getReefCenter(), getRobotPose).getDegrees();

    // Rounding to the nearest 60 degrees (the reef is hexagontal)
    return Rotation2d.fromDegrees(Math.round(angleDegrees / 60.0) * 60.0);
  }

  /**
   * @return the heading to point at the coral station corresponding
   *  to the side of the field the robot is on.
   */
  public static Rotation2d getPointAtCoralStationAngle(Supplier<Pose2d> getRobotPose) {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      return getTargetingAngle(
        getReefCenter(),
        getRobotPose
      ).getDegrees() > 0.0 ? TeleopFeature.RED_LEFT_CORAL_STATION_ANGLE : TeleopFeature.RED_RIGHT_CORAL_STATION_ANGLE;

    } else {
      return getTargetingAngle(
        getReefCenter(),
        getRobotPose
      ).getDegrees() < 0.0 ? TeleopFeature.BLUE_LEFT_CORAL_STATION_ANGLE : TeleopFeature.BLUE_RIGHT_CORAL_STATION_ANGLE;
    }
  }

  public static Translation2d getReefCenter() {
    return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) ? 
      TeleopFeature.REEF_RED : TeleopFeature.REEF_BLUE;
  }
}
