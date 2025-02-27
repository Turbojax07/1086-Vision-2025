// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.util;

import org.photonvision.targeting.MultiTargetPNPResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionConstants.GeneralConstants;

/** Add your docs here. */
public class VisionFunctions {
    public static Pose3d calculateMultiTagResult(MultiTargetPNPResult result, Transform3d robotToCameraTransform) {
        Transform3d fieldToCamera = result.estimatedPose.best;
        Pose3d estimatedPose = new Pose3d().plus(fieldToCamera).relativeTo(AprilTagFieldLayout.loadField(GeneralConstants.field).getOrigin()).plus(robotToCameraTransform.inverse());
        return estimatedPose;
    }
}
