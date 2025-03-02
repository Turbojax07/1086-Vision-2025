// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.io;

import frc.robot.subsystems.vision.VisionConstants;
import java.util.List; 
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.util.VisionResult;

/** Add your docs here. */
public class VisionIO_REAL implements VisionIO {
    PhotonCamera[] cameras;
    PhotonPoseEstimator[] poseEstimators;

    VisionIOInputsAutoLogged inputs;

    public VisionIO_REAL() {
        cameras = new PhotonCamera[VisionConstants.CameraIDs.length];
        poseEstimators = new PhotonPoseEstimator[cameras.length];

        for (int i=0; i<cameras.length; i++) {
            cameras[i] = new PhotonCamera(VisionConstants.CameraIDs[i]);
            poseEstimators[i] = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(VisionConstants.field), VisionConstants.strategy, VisionConstants.CameraTransforms[i]);
        }

        inputs = new VisionIOInputsAutoLogged();
    }

    @Override
    public void updateInputs() {
        inputs.unreadResults = getUnreadResults();

        Logger.processInputs("/Vision/Cameras", inputs);
    }

    @Override
    public VisionResult[] getUnreadResults() {
        Logger.recordOutput("Cameras/Measuring", true);
        VisionResult[] visionMeasurements = new VisionResult[cameras.length];
        for (int i=0; i<cameras.length; i++) {
            List<PhotonPipelineResult> results = cameras[i].getAllUnreadResults();
            if (results.size() > 0) {
                Optional<EstimatedRobotPose> estimatedPose = poseEstimators[i].update(results.get(0));
                if (estimatedPose.isPresent()) {
                    visionMeasurements[i] = new VisionResult(estimatedPose.get().estimatedPose, results.get(0).getTimestampSeconds());
                }
            }
        }

        return visionMeasurements;
    }

	@Override
	public void setRobotPose(Pose2d pose) {}
}
