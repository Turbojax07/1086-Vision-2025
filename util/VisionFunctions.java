package frc.robot.subsystems.vision.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionFunctions {
    public static Pose3d calculateMultiTagResult(MultiTargetPNPResult result, Transform3d robotToCameraTransform) {
        Transform3d fieldToCamera = result.estimatedPose.best;

        return new Pose3d().plus(fieldToCamera).relativeTo(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getOrigin()).plus(robotToCameraTransform.inverse());
    }

    public static Matrix<N3, N1> getStdDevs(PhotonCamera camera, Pose2d estimatedPose, PhotonPoseEstimator poseEstimator) {
        Matrix<N3,N1> estStdDevs = VisionConstants.singleTagStdDevs;

        List<PhotonTrackedTarget> targets = camera.getAllUnreadResults().get(0).targets;
        int numTags = 0;
        double avgDist = 0;
        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;

            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }

        if (numTags == 0) return estStdDevs;

        avgDist /= numTags;

        if (numTags > 1) estStdDevs = VisionConstants.multiTagStdDevs;

        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        return estStdDevs;
    }
}