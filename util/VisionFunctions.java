// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.util;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionConstants.GeneralConstants;

import static frc.robot.subsystems.vision.VisionConstants.GeneralConstants.*;

/** Add your docs here. */
public class VisionFunctions {
    public static Pose3d calculateMultiTagResult(MultiTargetPNPResult result, Transform3d robotToCameraTransform) {
        Transform3d fieldToCamera = result.estimatedPose.best;
        Pose3d estimatedPose = new Pose3d().plus(fieldToCamera).relativeTo(AprilTagFieldLayout.loadField(GeneralConstants.field).getOrigin()).plus(robotToCameraTransform.inverse());
        return estimatedPose;
    }

    public static Matrix<N3, N1> getStdDevs(PhotonCamera camera, Pose2d estimatedPose, PhotonPoseEstimator poseEstimator) {
        Matrix<N3, N1> estStdDevs = singleTagStdDevs;
        List<PhotonTrackedTarget> targets = camera.getLatestResult().targets;
        int numTags = 0;
        double avgDist = 0;
        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(target.fiducialId);
            if (tagPose.isEmpty()) {
                continue;
            }
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }

        if (numTags == 0) {
            return estStdDevs;
        }

        avgDist /= numTags;
        if (numTags > 1) {
            estStdDevs = multiTagStdDevs;
        }

        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + ((avgDist*avgDist)/30));
        }
        
        return estStdDevs;
    }
}
