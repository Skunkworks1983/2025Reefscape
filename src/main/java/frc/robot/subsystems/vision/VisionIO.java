package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {

    record PoseObservation(
        double timestamp,
        Pose3d estimatedPose,
        double ambiguity,
        int tagCount,
        double averageTagDistance) {
    }

    public class VisionIOData {
        public List<PoseObservation> poseObservations = new LinkedList<PoseObservation>();
    }

    public VisionIOData getLatestData();
    public String getName();
}
