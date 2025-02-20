package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.VisionIOConstantsPhotonVision;

public class VisionIOPhotonVision implements VisionIO {

    private final String cameraName;
    private final Transform3d robotToCamera;
    private final PhotonCamera camera;
    private final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public VisionIOPhotonVision(VisionIOConstantsPhotonVision constants) {
      this(constants.cameraName, constants.robotToCamera);
    }

    // public VisionIOPhotonVision() {
    //   this.camera= new PhotonCamera("cameraName");
    //   this.robotToCamera = new Transform3d();
    //   this.cameraName = "";
    // }

    public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
      this.cameraName = cameraName;
      this.robotToCamera = robotToCamera;
      this.camera = new PhotonCamera(cameraName);
    }

    @Override
    public VisionIOData getLatestData() {
      VisionIOData latestData = new VisionIOData();

      for (PhotonPipelineResult result : camera.getAllUnreadResults()) {

        if (result.getMultiTagResult().isPresent()) {
          MultiTargetPNPResult multitagResult = result.getMultiTagResult().get();

          Transform3d fieldToCamera = multitagResult.estimatedPose.best;
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d estimatedRobotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          double totalTagDistance = 0.0;
          for (PhotonTrackedTarget target : result.targets) {
            totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
          }

          latestData.poseObservations.add(
            new PoseObservation(
              result.getTimestampSeconds(),
              estimatedRobotPose,
              multitagResult.estimatedPose.ambiguity,
              multitagResult.fiducialIDsUsed.size(),
              totalTagDistance / result.targets.size()
            )
          );

          } else if (!result.targets.isEmpty()) {
            PhotonTrackedTarget target = result.targets.get(0);
            Optional<Pose3d> tagPose = aprilTagLayout.getTagPose(target.fiducialId);

            if (tagPose.isPresent()) {
              Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(),
              tagPose.get().getRotation());

              Transform3d cameraToTarget = target.bestCameraToTarget;
              Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
              Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
              Pose3d estimatedRobotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

              latestData.poseObservations.add(
                new PoseObservation(
                  result.getTimestampSeconds(),
                  estimatedRobotPose,
                  target.poseAmbiguity,
                  1,
                  cameraToTarget.getTranslation().getNorm()
                )
              );
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
