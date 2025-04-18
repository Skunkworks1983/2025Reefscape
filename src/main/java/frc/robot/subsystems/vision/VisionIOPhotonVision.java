package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhotonVision implements VisionIO{

  private final String cameraName;
  private final Transform3d robotToCamera;
  private final PhotonCamera camera;
  private final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  private final double kAngularStdDev = 10.0;

  public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
    this.cameraName = cameraName;
    this.robotToCamera = robotToCamera;
    this.camera = new PhotonCamera(cameraName);
  }

  @Override
  public List<VisionMeasurement> getUnreadMeasurements() {

    List<VisionMeasurement> measurements = new LinkedList<VisionMeasurement>();

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {

      for (PhotonTrackedTarget target : result.getTargets()) {
        Optional<Pose3d> tagPose = aprilTagLayout.getTagPose(target.getFiducialId());

        if (tagPose.isPresent()) {
          Pose3d fieldToTarget = tagPose.get();
          Transform3d cameraToTarget = target.getBestCameraToTarget();
          Pose3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Pose3d robotPose = fieldToCamera.plus(robotToCamera.inverse());
          double distToTag = cameraToTarget.getTranslation().getNorm();

          // Reject the pose if it's not within the field boundries.
          boolean rejectPose = 
            robotPose.getX() < 0.0 ||
            robotPose.getX() > aprilTagLayout.getFieldLength() ||
            robotPose.getY() < 0.0 ||
            robotPose.getY() > aprilTagLayout.getFieldWidth();

          if (rejectPose) {
            continue;
          }
  
          double translationStdDev = distToTag;
          
          measurements.add(
            new VisionMeasurement(
              robotPose, 
              result.getTimestampSeconds(), 
              VecBuilder.fill(translationStdDev, translationStdDev, kAngularStdDev)
            )
          );
        }
      }
    }

    return measurements;
  }




  @Override
  public String getName() {
    return cameraName;
  }
}
