package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionIOPhotonVision implements VisionIO {

    private final PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator poseEstimator;

    public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);

        try {
            aprilTagFieldLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Exception loading AprilTag field layout JSON: " + e.toString());
        }

        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    }

    @Override
    public VisionIOData getLatestData() {
        VisionIOData latest = new VisionIOData();

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            
            if (result.hasTargets()) {
                Optional<EstimatedRobotPose> updatedPose = poseEstimator.update(result);

                if (updatedPose.isPresent()) {
                    EstimatedRobotPose pose = updatedPose.get();

                    if (pose.estimatedPose.toPose2d().getX() > 0.0
                            && pose.estimatedPose.toPose2d().getX() < 20
                            && pose.estimatedPose.toPose2d().getY() > 0.0
                            && pose.estimatedPose.toPose2d().getY() < 20) {

                        double timestamp = pose.timestampSeconds;
                        Pose2d pose2d = pose.estimatedPose.toPose2d();

                        // TODO: Figure out how to handle uncertainty
                        double[] stdDevs = { 0 };
                        PoseObservation poseObservation = new PoseObservation(timestamp, pose2d,
                                new Matrix<N3, N1>(new SimpleMatrix(stdDevs)));
                        latest.poseObservations.add(poseObservation);
                    }
                }
            }
        }
        
        return latest;
    }
}
