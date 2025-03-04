package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.util.VisionResult;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    CameraIO[] cameraIOs;

    /** Creates a new Vision system. */
    public Vision(CameraIO... cameraIOs) {
        this.cameraIOs = cameraIOs;
    }

    @Override
    public void periodic() {
        update(new Pose2d());

        VisionResult[] measuredPoses = getUnreadResults();

        for (int i = 0; i < measuredPoses.length; i++) {
            VisionResult measuredPose = measuredPoses[i];

            if (measuredPose != null) Logger.recordOutput(String.format("/Cameras/Camera%d/Estimated_Pose", i), measuredPose.getPose2d());
        }
    }

    public VisionResult[] getUnreadResults() {
        ArrayList<VisionResult> results = new ArrayList<VisionResult>();

        for (int i = 0; i < cameraIOs.length; i++) {
            CameraIO cameraIO = cameraIOs[i];

            for (VisionResult result : cameraIO.getUnreadResults()) {
                results.add(result);
            }
        }

        return (VisionResult[]) results.toArray();
    }

    public void update(Pose2d pose) {
        for (CameraIO cameraIO : cameraIOs) {
            cameraIO.setRobotPose(pose);
        }
    }
}
