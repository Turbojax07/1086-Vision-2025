package frc.robot.subsystems.vision.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.struct.StructSerializable;

public class VisionResult implements StructSerializable {
    private Pose3d pose3d;
    private double timestamp;
    private Matrix<N3, N1> stdDevs;

    /**
     * Creates a new VisionResult with no standard deviations.
     * 
     * @param pose The pose to record
     * @param timestamp The time that the robot was at this pose.
     */
    public VisionResult(Pose3d pose, double timestamp) {
        this(pose, timestamp, null);
    }

    /**
     * Creates a new VisionResult with standard deviations.
     * 
     * @param pose The pose to record.
     * @param timestamp The time that the robot was at this pose.
     * @param stdDevs The standard deviations to use.
     */
    public VisionResult(Pose3d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        this.pose3d = pose;
        this.timestamp = timestamp;
        this.stdDevs = stdDevs;
    }

    /** Gets the {@link Pose3d} of the vision result. */
    public Pose3d getPose3d() {
        return pose3d;
    }

    /** Gets the {@link Pose2d} of the vision result. */
    public Pose2d getPose2d() {
        return pose3d.toPose2d();
    }

    /** Gets the timestamp of the vision result. */
    public double getTimestamp() {
        return timestamp;
    }

    /** Gets the standard deviations of the vision result. */
    public Matrix<N3, N1> getStdDevs() {
        return stdDevs;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof VisionResult result) {
            return pose3d.equals(result.pose3d) &&
                   timestamp == result.timestamp &&
                   stdDevs.equals(result.stdDevs);
        }

        return false;
    }

    /** VisionResult struct for serialization. */
    public static final VisionResultStruct struct = new VisionResultStruct();
}