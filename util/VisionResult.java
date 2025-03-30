package frc.robot.subsystems.vision.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.struct.StructSerializable;

public class VisionResult implements StructSerializable {
    private Pose3d pose;
    private double timestamp;
    private static Matrix<N3,N1> stdDevs;

    /** Creates a new VisionResult with no standard deviations. */
    public VisionResult(Pose3d pose, double timestamp) {
        this(pose, timestamp, null);
    }

    /** Creates a new VisionResult with standard deviations. */
    public VisionResult(Pose3d pose, double timestamp, Matrix<N3,N1> stdDevs) {
        this.pose = pose;
        this.timestamp = timestamp;
        VisionResult.stdDevs = stdDevs;
    }

    /** Gets the {@link Pose3d} of the vision result. */
    public Pose3d getPose3d() {
        return pose;
    }

    /** Gets the {@link Pose2d} of the vision result. */
    public Pose2d getPose2d() {
        return pose.toPose2d();
    }

    /** Gets the timestamp of the vision result. */
    public double getTimestamp() {
        return timestamp;
    }

    /** Gets the standard deviations of the vision result. */
    public static Matrix<N3,N1> getStdDevs() {
        return stdDevs;
    }
}