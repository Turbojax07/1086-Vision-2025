// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.io;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionConstants.GeneralConstants;
import frc.robot.subsystems.vision.util.VisionFunctions;
import frc.robot.subsystems.vision.util.VisionResult;

/** Add your docs here. */
public class VisionIO_SIM implements VisionIO {
    PhotonCamera[] cameras;
    PhotonCameraSim[] simCameras;
    VisionSystemSim visionSim;
    PhotonPoseEstimator[] poseEstimators;
    double[] targetYaw;

    public VisionIO_SIM() {
        SimCameraProperties camProperties = new SimCameraProperties();
        camProperties.setCalibration(320, 320, Rotation2d.fromDegrees(70));
        // camProperties.setCalibError(0.25, 0.08);
        camProperties.setFPS(90);
        // camProperties.setAvgLatencyMs(35);
        // camProperties.setLatencyStdDevMs(5);
        cameras = new PhotonCamera[GeneralConstants.CameraIDs.length];
        simCameras = new PhotonCameraSim[GeneralConstants.CameraIDs.length];
        poseEstimators = new PhotonPoseEstimator[cameras.length];
        visionSim = new VisionSystemSim("visionSim");
        visionSim.addAprilTags(AprilTagFieldLayout.loadField(GeneralConstants.field));
        for (int i=0; i<cameras.length; i++) {
            cameras[i] = new PhotonCamera(GeneralConstants.CameraIDs[i]);
            simCameras[i] = new PhotonCameraSim(cameras[i], camProperties);
            poseEstimators[i] = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(GeneralConstants.field), GeneralConstants.strategy, GeneralConstants.CameraTransforms[i]);
            // poseEstimators[i].setMultiTagFallbackStrategy(GeneralConstants.fallbackStrategy);
            visionSim.addCamera(simCameras[i], poseEstimators[i].getRobotToCameraTransform());
        }
        targetYaw = new double[22];
    }

    @Override
    public VisionResult[] getMeasurements() {
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
	public void update(Pose2d pose) {
		visionSim.update(pose);
        for (int i=0; i<poseEstimators.length; i++) {
            poseEstimators[i].setReferencePose(pose);
        }
       
        Pose3d[] targetPoses = new Pose3d[22];
        PhotonTrackedTarget[][] cameraTargets = new PhotonTrackedTarget[cameras.length][] ;
        for (PhotonCamera camera : cameras) {
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
            if (results.size() > 0) {
                for (int i=0; i<results.size(); i++) {
                    List<PhotonTrackedTarget> targets = results.get(i).getTargets();
                    PhotonTrackedTarget[] trackedTargets = new PhotonTrackedTarget[targets.size()];
                    if (targets.size() > 0) {
                        for (int t=0; t<targets.size(); t++) {
                            Optional<Pose3d> targetPose = AprilTagFieldLayout.loadField(GeneralConstants.field).getTagPose(targets.get(t).getFiducialId());
                            trackedTargets[t] = targets.get(t);
                            if (targetPose.isPresent()) {
                                targetPoses[targets.get(t).fiducialId-1] = targetPose.get();
                            }   
                        }
                    }
                }
            } 
        }
        for (int i=0; i<targetPoses.length; i++) {
            if (targetPoses[i] == null) {
                targetPoses[i] = new Pose3d(pose);
            }
        }
        Logger.recordOutput("Target Poses", targetPoses);
	}
    

    @Override
    public double[] getTagYaw() {
        return targetYaw;
    }
}
