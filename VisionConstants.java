package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionConstants {
    public static final String lCameraName = "CamLeft";
    public static final String rCameraName = "CamRight";

    public static final Transform3d lCameraTransform = new Transform3d(0.2136, 0.2864, 0.2085, new Rotation3d(0, -Math.PI / 5,  Math.PI / 9));
    public static final Transform3d rCameraTransform = new Transform3d(0.2136, 0.2864, 0.2085, new Rotation3d(0, -Math.PI / 5, -Math.PI / 9));

    public static final PoseStrategy strategy = PoseStrategy.LOWEST_AMBIGUITY;
    public static final AprilTagFields field = AprilTagFields.kDefaultField;

    public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(2, 2, 8);
    public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.1, 0.1, 1);
}
