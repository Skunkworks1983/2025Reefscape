package frc.robot.subsystems.vision;

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
        public PoseObservation[] poseObservations = new PoseObservation[0];
    }

    public VisionIOData getLatestData();
}
