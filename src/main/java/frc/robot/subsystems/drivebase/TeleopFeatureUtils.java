
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
    // Using Math.floor() here because Math.round() is unintuitive when the value is in the middle, like .5
    return Rotation2d.fromDegrees(Math.floor((angleDegrees + 30) / 60) * 60);
  }

  /**
   * @return the heading to point at the coral station corresponding
   *  to the side of the field the robot is on.
   */
  public static Rotation2d getPointAtCoralStationAngle(Supplier<Pose2d> getRobotPose) {
    return (getTargetingAngle(
      getReefCenter(),
      getRobotPose
    ).getDegrees() > 0 ?  
      TeleopFeature.LEFT_CORAL_STATION_ANGLE : TeleopFeature.RIGHT_CORAL_STATION_ANGLE).rotateBy(Rotation2d.k180deg);
  }

  public static Translation2d getReefCenter() {
    return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) ? 
      TeleopFeature.REEF_RED : TeleopFeature.REEF_BLUE;
  }
}
