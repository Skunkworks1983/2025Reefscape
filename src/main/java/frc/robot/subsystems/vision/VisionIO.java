package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface VisionIO {

    record PoseObservation(
        double timestamp,
        Pose2d estimatedPose,
        Matrix<N3, N1> stdDevs) {
    }


    public class VisionIOData {
        public List<PoseObservation> poseObservations = new LinkedList<PoseObservation>();
    }

    public VisionIOData getLatestData();
    public String getName();
}
