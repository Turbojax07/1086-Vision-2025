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
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionConstants.GeneralConstants;
import frc.robot.subsystems.vision.util.VisionResult;
import static frc.robot.subsystems.vision.util.VisionFunctions.getStdDevs;

/** Add your docs here. */
public class VisionIO_REAL implements VisionIO {
    PhotonCamera[] cameras;
    PhotonPoseEstimator[] poseEstimators;
    double[] targetYaw;
    public VisionIO_REAL() {
        cameras = new PhotonCamera[GeneralConstants.CameraIDs.length];
        poseEstimators = new PhotonPoseEstimator[cameras.length];
        for (int i=0; i < cameras.length; i++) {
            cameras[i] = new PhotonCamera(GeneralConstants.CameraIDs[i]);
            poseEstimators[i] = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(GeneralConstants.field), GeneralConstants.strategy, GeneralConstants.CameraTransforms[i]);
        }
        targetYaw = new double[22];
    }

    

    @Override
    public VisionResult[] getMeasurements() {
        VisionResult[] visionMeasurement = new VisionResult[cameras.length];
        for (int i=0; i < cameras.length; i++) {
            if (cameras[i] != null) {
                // +-----------------------+
                // | Do not  use this code |
                // +-----------------------+
                // List<PhotonPipelineResult> pipelineResults = cameras[i].getAllUnreadResults();
                // if (pipelineResults.size() > 0) {
                //     // may be last elem
                //     Optional<EstimatedRobotPose> estimatedPose = poseEstimators[i].update(pipelineResults.get(0));
                //     if (estimatedPose.isPresent()) {
                //         visionMeasurement[i] = new VisionResult(estimatedPose.get().estimatedPose, Timer.getFPGATimestamp());
                //     }
                // }
                PhotonPipelineResult result = cameras[i].getLatestResult();
                Optional<EstimatedRobotPose> estimatedPose = poseEstimators[i].update(result);
                if (estimatedPose.isPresent()) {
                    visionMeasurement[i] = new VisionResult(estimatedPose.get().estimatedPose, Timer.getFPGATimestamp(), getStdDevs(cameras[i], estimatedPose.get().estimatedPose.toPose2d(), poseEstimators[i]));
                }
            }
        }
        return visionMeasurement;
    }

    @Override
    public void update(Pose2d pose) {
        Pose3d[] targetPoses = new Pose3d[22];
        for (int i=0; i<cameras.length; i++) {
            PhotonCamera camera = cameras[i];
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
            if (results.size() > 0) {
                for (int f=0; f<results.size(); f++) {
                    List<PhotonTrackedTarget> targets = results.get(f).getTargets();
                    PhotonTrackedTarget[] trackedTargets = new PhotonTrackedTarget[targets.size()];
                    if (targets.size() > 0) {
                        for (int t=0; t<targets.size(); t++) {
                            Optional<Pose3d> targetPose = AprilTagFieldLayout.loadField(GeneralConstants.field).getTagPose(targets.get(t).getFiducialId());
                            trackedTargets[t] = targets.get(t);
                            targetYaw[targets.get(t).fiducialId-1] = targets.get(t).yaw;
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



    @Override
    public PhotonPipelineResult getLatestCameraResult(int cameraIndex) {
        if (cameraIndex > cameras.length-1) {
            throw new Error("Camera Index is greater than length");
        }
        if (cameraIndex < 0) {
            throw new Error("Camera Index is less than 0");
        }
        return cameras[cameraIndex].getLatestResult();
    }
}
