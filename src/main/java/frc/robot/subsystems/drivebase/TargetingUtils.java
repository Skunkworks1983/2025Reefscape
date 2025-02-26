
package frc.robot.subsystems.drivebase;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants.Drivebase.FieldTarget;

public class TargetingUtils {

  /**
   * @param target the field location to point at.
   * @return the heading needed for the robot to point at the target.
   */
  public static Rotation2d getTargetingAngle(Translation2d target, Supplier<Pose2d> getRobotPose) {
    Pose2d robotPose = getRobotPose.get();
    Rotation2d angle = new Rotation2d(target.getX() - robotPose.getX(), target.getY() - robotPose.getY());
    return angle;
  }

  /** @return the angle needed to point at the nearest face of the reef */
  public static Rotation2d getPointAtReefFaceAngle(Supplier<Pose2d> getRobotPose) {
    double angleDegrees = getTargetingAngle(FieldTarget.REEF_BLUE, getRobotPose).getDegrees();
    // Using Math.floor here because Math.round is unintuitive when the value is in the middle, like .5
    return Rotation2d.fromDegrees(Math.floor((angleDegrees + 30) / 60) * 60);
  }

  /**
   * @return the heading to point at the coral station corresponding
   *  to the side of the field the robot is on.
   */
  public static Rotation2d getPointAtCoralStationAngle(Supplier<Pose2d> getRobotPose) {
    return getTargetingAngle(
      FieldTarget.REEF_BLUE,
      getRobotPose
    ).getDegrees() > 0 ?  
      FieldTarget.LEFT_CORAL_STATION_ANGLE : FieldTarget.RIGHT_CORAL_STATION_ANGLE;
  }
}
