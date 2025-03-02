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
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.util.VisionResult;

/** Add your docs here. */
public class VisionIO_SIM implements VisionIO {
    PhotonCamera[] cameras;
    PhotonCameraSim[] simCameras;
    VisionSystemSim visionSim;
    PhotonPoseEstimator[] poseEstimators;

    VisionIOInputsAutoLogged inputs;

    public VisionIO_SIM() {
        SimCameraProperties camProperties = new SimCameraProperties();
        camProperties.setCalibration(320, 320, Rotation2d.fromDegrees(70));
        // camProperties.setCalibError(0.25, 0.08);
        camProperties.setFPS(90);
        // camProperties.setAvgLatencyMs(35);
        // camProperties.setLatencyStdDevMs(5);
        cameras = new PhotonCamera[VisionConstants.CameraIDs.length];
        simCameras = new PhotonCameraSim[VisionConstants.CameraIDs.length];
        poseEstimators = new PhotonPoseEstimator[cameras.length];
        visionSim = new VisionSystemSim("visionSim");
        visionSim.addAprilTags(AprilTagFieldLayout.loadField(VisionConstants.field));
        for (int i=0; i<cameras.length; i++) {
            cameras[i] = new PhotonCamera(VisionConstants.CameraIDs[i]);
            simCameras[i] = new PhotonCameraSim(cameras[i], camProperties);
            poseEstimators[i] = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(VisionConstants.field), VisionConstants.strategy, VisionConstants.CameraTransforms[i]);
            visionSim.addCamera(simCameras[i], poseEstimators[i].getRobotToCameraTransform());
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
        VisionResult[] visionMeasurements = new VisionResult[simCameras.length];
        for (int i=0; i<simCameras.length; i++) {
            List<PhotonPipelineResult> results = simCameras[i].getCamera().getAllUnreadResults();
            if (results.size() > 0) {
                Optional<EstimatedRobotPose> estimatedPose = poseEstimators[i].update(results.get(0));
                if (estimatedPose.isPresent()) {
                    visionMeasurements[i] = new VisionResult(estimatedPose.get().estimatedPose, results.get(0).getTimestampSeconds());
                }
            }
            // Below is what we should use a fallback code otherwise use the code above unless it is absolutely necessary to use this code
            // PhotonPipelineResult pipelineResult = simCameras[i].getCamera().getLatestResult();
            // Optional<EstimatedRobotPose> pose = poseEstimators[i].update(pipelineResult);
            // if (pose.isPresent()) {
            //     visionMeasurements[i] = new VisionResult(pose.get().estimatedPose, pose.get().timestampSeconds);
            // }
            }
        return visionMeasurements;
    }

	@Override
	public void setRobotPose(Pose2d pose) {
		visionSim.update(pose);
	}
}
