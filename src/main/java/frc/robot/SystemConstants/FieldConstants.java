// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SystemConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class FieldConstants {

    public class BlueAlliance {
        public static final double ScoringXPose = 0.17;
        public static final double ScoringYPose = 5.55;
        public static final Pose2d ScoringPose = new Pose2d(ScoringXPose, ScoringYPose, new Rotation2d(0));
    }

    public class RedAlliance {
        public static final double ScoringXPose = 16.38;
        public static final double ScoringYPose = 5.55;
        public static final Pose2d ScoringPose = new Pose2d(ScoringXPose, ScoringYPose, new Rotation2d(0));
    }
}
