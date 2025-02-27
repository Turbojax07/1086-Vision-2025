// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.io;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.util.VisionResult;

/** Add your docs here. */
public interface VisionIO {

  public abstract VisionResult[] getMeasurements();
  public abstract void update(Pose2d presetPose);
  public abstract double[] getTagYaw();
  public abstract PhotonPipelineResult getLatestCameraResult(int cameraIndex);
}
