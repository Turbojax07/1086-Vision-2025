package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.io.VisionIO;
import frc.robot.subsystems.vision.io.VisionIOInputsAutoLogged;
import frc.robot.subsystems.vision.util.VisionResult;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private VisionIO io;

    /** Creates a new Vision. */
    public Vision(VisionIO io) {
        this.io = io;
    }

    public VisionResult[] getVisionMeasurements() {
        return io.getMeasurements();
    }

    public void update(Pose2d pose) {
        io.update(pose);
    }

    @Override
    public void periodic() {
        VisionResult[] measuredPoses = getVisionMeasurements();
        Logger.recordOutput("Cameras/Active", true);
        for (int i=0; i<measuredPoses.length; i++) {
            if (measuredPoses[i] != null) {
                Logger.recordOutput("Cameras/Camera #"+(i+1)+" Estimated Pose", measuredPoses[i].getPose2d());
            } else {
                Logger.recordOutput("Cameras/Camera #"+(i+1)+" Estimated Pose", new Pose2d());
            }
        }
    }
}
