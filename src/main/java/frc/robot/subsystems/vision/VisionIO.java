package frc.robot.subsystems.vision;

import java.lang.annotation.Target;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {

  public class VisionIOData {
    public boolean connected = false;
    public List<PoseObservation> poseObservations = new LinkedList<PoseObservation>();
  }

  public class ObjectDetectionData {
    public TargetObservation lastObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
  }

  record PoseObservation(
    double timestamp,
    Pose3d estimatedPose,
    double ambiguity,
    int tagCount,
    double averageTagDistance) {
  }

  record TargetObservation(
    Rotation2d tx,
    Rotation2d ty) {
  }

  public VisionIOData getLatestData();
  public String getName();
}
