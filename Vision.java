package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.util.VisionResult;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private CameraIO[] cameraIOs;
    private CameraIOInputsAutoLogged[] cameraInputs;

    /** Creates a new Vision system. */
    public Vision(CameraIO... cameraIOs) {
        this.cameraIOs = cameraIOs;

        cameraInputs = new CameraIOInputsAutoLogged[cameraIOs.length];

        for (int i = 0; i < cameraIOs.length; i++) {
            cameraInputs[i] = new CameraIOInputsAutoLogged();
        }
    }

    /**
     * Runs once every tick the subsystem is active.
     * 
     * It updates the camera IO Inputs and logs the estimated pose for each camera
     */
    @Override
    public void periodic() {
        for (int i = 0; i < cameraIOs.length; i++) {
            CameraIO cameraIO = cameraIOs[i];

            cameraIO.updateInputs(cameraInputs[i]);

            VisionResult[] unreadResults = cameraInputs[i].unreadResults;

            Logger.processInputs("/RealOutputs/Vision/" + cameraInputs[i].cameraName, cameraInputs[i]);

            Logger.recordOutput("/Vision/" + cameraInputs[i].cameraName + "/LatestPose", unreadResults[unreadResults.length].getPose2d());
        }
    }

    /** Gets all of the unread results for each camera. */
    public VisionResult[] getUnreadResults() {
        ArrayList<VisionResult> allResults = new ArrayList<VisionResult>();

        for (int i = 0; i < cameraInputs.length; i++) {
            for (VisionResult result : cameraInputs[i].unreadResults) {
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
        for (CameraIO cameraIO : cameraIOs) {
            cameraIO.setRobotPose(pose);
        }
    }
}