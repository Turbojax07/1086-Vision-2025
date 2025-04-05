package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.util.VisionFunctions;
import frc.robot.subsystems.vision.util.VisionResult;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraIOReal implements CameraIO {
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;

    public CameraIOReal(String cameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);

        poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                VisionConstants.strategy, robotToCamera);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        ArrayList<VisionResult> visionResults = new ArrayList<VisionResult>(results.size());

        for (int i = 0; i < results.size(); i++) {
            Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(results.get(i));

            if (estimatedPose.isEmpty()) continue;

            visionResults.add(new VisionResult(
                    estimatedPose.get().estimatedPose,
                    estimatedPose.get().timestampSeconds,
                    VisionFunctions.getStdDevs(results.get(i), estimatedPose.get().estimatedPose,
                            poseEstimator.getFieldTags())));
        }

        inputs.cameraName = camera.getName();
        inputs.isActive = camera.isConnected();
        inputs.unreadResults = visionResults.toArray(new VisionResult[0]);
    }

    @Override
    public void setRobotPose(Pose2d pose) {}
}