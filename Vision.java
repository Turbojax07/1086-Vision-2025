package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.util.VisionResult;
import frc.robot.util.TurboLogger;
import java.util.ArrayList;

public class Vision extends SubsystemBase {
    private Camera[] cameras;

    /** Creates a new Vision system. */
    public Vision(Camera... cameras) {
        this.cameras = cameras;
    }

    /**
     * Runs once every tick the subsystem is active.
     *
     * <p>It updates the camera IO Inputs and logs the estimated pose for each camera
     */
    @Override
    public void periodic() {
        for (int i = 0; i < cameras.length; i++) {
            VisionResult[] unreadResults = cameras[i].getUnreadResults();

            if (unreadResults.length == 0) return;

            TurboLogger.log("/Vision/" + cameras[i].getCameraName() + "/LatestPose", unreadResults[unreadResults.length - 1].getPose2d());
        }
    }

    /** Gets all of the unread results for each camera. */
    public VisionResult[] getUnreadResults() {
        ArrayList<VisionResult> allResults = new ArrayList<VisionResult>();

        for (int i = 0; i < cameras.length; i++) {
            for (VisionResult result : cameras[i].getUnreadResults()) {
                allResults.add(result);
            }
        }

        return allResults.toArray(new VisionResult[0]);
    }

    /**
     * Updates the cameras with the current pose of the robot.
     *
     * @param pose The pose of the robot.
     */
    public void update(Pose2d pose) {
        for (Camera camera : cameras) {
            camera.setRobotPose(pose);
        }
    }
}
