package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

/** 
 * Represents one camera. 
 */
public interface VisionIO {

  public class IOData {
    public boolean connected = false;
  }

  public class PoseEstimationData {
    public List<PoseObservation> poseObservations = new LinkedList<PoseObservation>();
  }

  public class ColoredShapeData {
    public ColoredShapeObservation lastObservation = new ColoredShapeObservation(new Transform3d());
  }

  record PoseObservation(
    double timestamp,
    Pose3d estimatedPose,
    double ambiguity,
    int tagCount,
    double averageTagDistance) {
  }

  record ColoredShapeObservation(
    Transform3d robotToTarget) {
  }

  public PoseEstimationData getLatestData();
  public String getName();
}
