// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.util;

import java.util.List;

import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class VisionResult {
    Pose3d pose;
    double timeStamp;

    public VisionResult(Pose3d pose, double timeStamp) {
        this.pose = pose;
        this.timeStamp = timeStamp;
    }

    public Pose2d getPose2d() {
        return pose.toPose2d();
    }
}
