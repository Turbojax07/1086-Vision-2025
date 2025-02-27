// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.util;

import java.util.List;

import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class VisionResult {
    Pose3d pose;
    double timeStamp;
    Matrix<N3, N1> stdDevs;

    public VisionResult(Pose3d pose, double timeStamp) {
        this.pose = pose;
        this.timeStamp = timeStamp;
    }


    public VisionResult(Pose3d pose, double timeStamp, Matrix<N3, N1> stdDevs) {
        this.pose = pose;
        this.timeStamp = timeStamp;
        this.stdDevs = stdDevs;
    }

    public Pose2d getPose2d() {
        return pose.toPose2d();
    }

    public double getTimestampSeconds() {
        return timeStamp;
    }

    public Matrix<N3,N1> getStdDevs() {
        return stdDevs;
    }
}
