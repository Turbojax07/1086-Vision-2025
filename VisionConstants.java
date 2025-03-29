// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class VisionConstants {
        public static final String lCameraName = "Camera_Module_v1_l";
        public static final String rCameraName = "Camera_Module_v1_r";

        public static final Transform3d lCameraTransform = new Transform3d(Inches.of(8.410427), Inches.of(11.276584),
                        Inches.of(8.209095), new Rotation3d(Degrees.zero(), Degrees.of(-36.053760), Degrees.of(20)));
        public static final Transform3d rCameraTransform = new Transform3d(Inches.of(8.410427), Inches.of(11.276584),
                        Inches.of(8.209095), new Rotation3d(Degrees.zero(), Degrees.of(-36.053760), Degrees.of(-20)));

        public static final PoseStrategy strategy = PoseStrategy.LOWEST_AMBIGUITY;
        public static final AprilTagFields field = AprilTagFields.kDefaultField;

        public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(2, 2, 8);
        public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.1, 0.1, 1);
}
