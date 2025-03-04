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

    public void updateInputs();

    public String getName();

    public VisionResult[] getUnreadResults();

    public boolean isConnected();

    public void setRobotPose(Pose2d robotPose);
}
