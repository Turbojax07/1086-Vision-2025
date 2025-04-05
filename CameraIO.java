package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.util.VisionResult;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
    @AutoLog
    public class CameraIOInputs {
        String cameraName;
        VisionResult[] unreadResults;
        boolean isActive;
    }

    /**
     * Updates a set of IO inputs with current values.
     * 
     * @param inputs The inputs to update.
     */
    public void updateInputs(CameraIOInputs inputs);

    /**
     * Updates the cameras with the current pose of the robot.
     * 
     * @param pose The pose of the robot.
     */
    public void setRobotPose(Pose2d robotPose);
}