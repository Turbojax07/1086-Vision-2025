// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.util.VisionResult;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface CameraIO {
    @AutoLog
    public class CameraIOInputs {
        String cameraName;
        VisionResult[] unreadResults;
        boolean isActive;
    }

    public void updateInputs();

    public String getName();
    
    public VisionResult[] getUnreadResults();

    public boolean isConnected();

    public void setRobotPose(Pose2d robotPose);
}