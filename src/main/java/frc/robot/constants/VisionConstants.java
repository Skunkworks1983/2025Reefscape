package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

  public class DoubleCameraMount {}
  public class TripleCameraMount {}
  public class SwerveModuleMount {}
  
  public static final String FRONT_CAMERA_NAME = "Camera_0";
    public static final String SIDE_CAMERA_NAME = "Camera_1";

    private static final Transform3d MOUNT_TO_FRONT_CAMERA = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(1.351),
            Units.inchesToMeters(-1.268),
            Units.inchesToMeters(-0.81)),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(-19.27),
            Units.degreesToRadians(-15.0)));

    private static final Transform3d MOUNT_TO_SIDE_CAMERA = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-1.050),
            Units.inchesToMeters(1.365078),
            Units.inchesToMeters(-0.762394)),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(-27.225),
            Units.degreesToRadians(97.0)));

    private static final Transform3d ROBOT_TO_MOUNT =
      new Transform3d(
        new Translation3d( // TODO: check these transformation estimations
          .305,
          .305,
          Units.inchesToMeters(8.25)
        ),
        new Rotation3d(
            0.0,
            0.0,
            0.0));

    public static final Transform3d ROBOT_TO_FRONT_CAMERA = ROBOT_TO_MOUNT.plus(MOUNT_TO_FRONT_CAMERA);
    public static final Transform3d ROBOT_TO_SIDE_CAMERA = ROBOT_TO_MOUNT.plus(MOUNT_TO_SIDE_CAMERA);

    public static final double MAX_AMBIGUITY = 0.3;
    public static final double LINEAR_STD_DEV_BASELINE = 0.02;
    public static final double ANGULAR_STD_DEV_BASELINE = 0.06;
    public static final double MAX_Z_ERROR = 3.0;
    public static final double MAX_AVERAGE_TAG_DISTANCE = 3.0; // Meters

    public static final int JITTER_TEST_ROLLING_AVG_LENGTH = 20;
}
