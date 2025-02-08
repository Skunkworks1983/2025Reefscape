package frc.robot.subsystems.vision;

import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PnpResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionIOPhotonVision implements VisionIO {

    private final String name;
    private final PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator poseEstimator;

    public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) throws Exception {

        name = cameraName;
        camera = new PhotonCamera(cameraName);
        
        aprilTagFieldLayout = AprilTagFieldLayout
            .loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);

        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    }

    @Override
    public VisionIOData getLatestData() {
        VisionIOData latest = new VisionIOData();

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {

            Transform3d fieldToCamera = result.getMultiTagResult().get().estimatedPose.best
            
            Optional<EstimatedRobotPose> updatedPose = poseEstimator.update(result);
            EstimatedRobotPose pose;

            // Get the optional.
            if (poseEstimator.update(result).isPresent()) {
                pose = updatedPose.get();
            } else {
                continue;
            }
            
            // Ensure that the estimated pose is within the field dimensions.
            if (pose.estimatedPose.toPose2d().getX() < 0.0
                || pose.estimatedPose.toPose2d().getX() > aprilTagFieldLayout.getFieldLength()
                || pose.estimatedPose.toPose2d().getY() < 0.0
                || pose.estimatedPose.toPose2d().getY() > aprilTagFieldLayout.getFieldWidth()) { 

                continue;
            }

            PnpResult e = result.multitagResult.get().estimatedPose;
            e.ambiguity

            Transform3d distanceToTargetTransform;

            // try/catch statement to ensure getBestCameraToTarget() won't crash code
            try {
                distanceToTargetTransform = result.getBestTarget().getBestCameraToTarget();
            } catch (NullPointerException e) {
                continue;
            }

            double timestamp = pose.timestampSeconds;
            Pose2d pose2d = pose.estimatedPose.toPose2d();
            double[] stdDevs = { .75, .75, 5 };
            PoseObservation poseObservation = new PoseObservation(timestamp, pose2d,
                new Matrix<N3, N1>(new SimpleMatrix(stdDevs)));
                latest.poseObservations.add(poseObservation);
        }
            
        
        return latest;
    }

    @Override
    public String getName() {
        return name;
    }
}
