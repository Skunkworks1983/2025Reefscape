package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.vision.VisionConstants;

public class VisionIOPhotonVision implements VisionIO{

  private final String cameraName;
  private final Transform3d robotToCamera;
  private final PhotonCamera camera;
  private final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
    this.cameraName = cameraName;
    this.robotToCamera = robotToCamera;
    this.camera = new PhotonCamera(cameraName);
  }

  @Override
  public VisionIOData getUnreadResults() {

    VisionIOData data = new VisionIOData();

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {

      for (PhotonTrackedTarget target : result.getTargets()) {
        Optional<Pose3d> tagPose = aprilTagLayout.getTagPose(target.getFiducialId());

        if (tagPose.isPresent()) {
          Pose3d fieldToTarget = tagPose.get();
          Transform3d cameraToTarget = target.getBestCameraToTarget();
          Pose3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Pose3d robotPose = fieldToCamera.plus(robotToCamera.inverse());

          boolean rejectPose = 
            robotPose.getX() < 0.0 ||
            robotPose.getX() > aprilTagLayout.getFieldLength() ||
            robotPose.getY() < 0.0 ||
            robotPose.getY() > aprilTagLayout.getFieldWidth();
  
          double tagDistance = 3.3;

          final double kA = 0.0329 + .005;
          final double kB = -0.0222 + .005;
          final double kC = 0.0048 + .005; // Adding a small value to constant term to decrease overall confidence
  
          // This is the
          double linearStdDev = (kA * Math.pow(x, 2)) + (kB * x) + kC;
          latestData.poseObservations.add(
              new PoseObservation(
                  // If the timestamp is in the future, then use the FPGA timestamp instead.
                  // Future timestamps can break the pose estimator.
                  Math.min(result.getTimestampSeconds(), Timer.getFPGATimestamp()),
                  fieldToRobot,
                  target.poseAmbiguity,
                  1,
                  cameraToTarget.getTranslation().getNorm()));
        }
      }
    }

    return latestData;
  }




  @Override
  public String getName() {
    return cameraName;
  }
}
