// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionConstants.GeneralConstants;
import frc.robot.subsystems.vision.io.VisionIO;
import frc.robot.subsystems.vision.io.VisionIO_REAL;
import frc.robot.subsystems.vision.io.VisionIO_SIM;
import frc.robot.subsystems.vision.util.VisionResult;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  Pose2d currentPose;
  VisionIO io;
  VisionResult[] lastResult;
  public Vision() {
    if (RobotBase.isReal()) {
      io = new VisionIO_REAL();
    } else {
      io = new VisionIO_SIM();
    }
    currentPose = new Pose2d();
    lastResult = new VisionResult[GeneralConstants.CameraIDs.length];
  }

  public VisionResult[] getVisionMeasurements() {
    return io.getMeasurements();
  }

  public void update(Pose2d pose) {
    currentPose = pose;
    io.update(pose);
  }

  public double getTargetYaw(int targetID) {
    double[] tagYaws = io.getTagYaw();
    return tagYaws[targetID-1];
  }

  @Override
  public void periodic() {
    update(new Pose2d());
    VisionResult[] measuredPoses = getVisionMeasurements();
    Logger.recordOutput("Cameras/Active", true);
    for (int i=0; i<measuredPoses.length; i++) {
      if (measuredPoses[i] != null) {
        Logger.recordOutput("Cameras/Camera #"+(i+1)+" Estimated Pose", measuredPoses[i].getPose2d());
      } else {
        if (lastResult[i] != null) {
          Logger.recordOutput("Cameras/Camera #"+(i+1)+" Estimated Pose", lastResult[i].getPose2d());
        } else {
          Logger.recordOutput("Cameras/Camera #"+(i+1)+" Estimated Pose", new Pose2d());
        }
      }
    }
    if (measuredPoses[0] == null && measuredPoses[0] == measuredPoses[1]) {
      Logger.recordOutput("Cameras/Deadzones", currentPose);
    } else {
      Logger.recordOutput("Cameras/Deadzones", new Pose2d(new Translation2d(100, 100), new Rotation2d()));
    }
    for (int i=0; i<measuredPoses.length; i++) {
      if (measuredPoses[i] == null && lastResult[i] != null) {
        measuredPoses[i] = lastResult[i];
      }
    }
    lastResult = measuredPoses;
  }
}
