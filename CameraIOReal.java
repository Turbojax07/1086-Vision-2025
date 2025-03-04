package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.util.VisionResult;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraIOReal implements CameraIO {
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;

    private CameraIOInputsAutoLogged inputs;

    public CameraIOReal(String cameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);

        poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField), VisionConstants.strategy, robotToCamera);

        inputs = new CameraIOInputsAutoLogged();
    }

    @Override
    public void updateInputs() {
        inputs.cameraName = getName();
        inputs.isActive = isConnected();
        inputs.unreadResults = getUnreadResults();
    }

    @Override
    public String getName() {
        return camera.getName();
    }

    @Override
    public VisionResult[] getUnreadResults() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        VisionResult[] visionResults = new VisionResult[results.size()];

        for (int i = 0; i < results.size(); i++) {
            Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(results.get(i));

            if (estimatedPose.isEmpty()) continue;

            visionResults[i] = new VisionResult(estimatedPose.get().estimatedPose, estimatedPose.get().timestampSeconds);
        }

        return visionResults;
    }

    @Override
    public boolean isConnected() {
        return camera.isConnected();
    }

    @Override
    public void setRobotPose(Pose2d pose) {}
}
