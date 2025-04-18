package frc.robot.subsystems.vision;

import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface VisionIO {

    record VisionMeasurement(
        Pose3d estimatedPose,
        double timestamp,
        Matrix<N3, N1> stdDevs) {
    }

    public List<VisionMeasurement> getUnreadMeasurements();
    public String getName();
}
