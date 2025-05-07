package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.util.VisionResult;

public class Camera extends SubsystemBase {
    public String getCameraName() {
        return "";
    }

    public boolean isConnected() {
        return false;
    }

    public VisionResult[] getUnreadResults() {
        return new VisionResult[0];
    }

    /**
     * Updates the cameras with the current pose of the robot.
     *
     * @param pose The pose of the robot.
     */
    public void setRobotPose(Pose2d robotPose) {}
}
