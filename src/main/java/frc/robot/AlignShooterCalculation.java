// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Distance;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.VisionSubsystem;

/** Add your docs here. */
public class AlignShooterCalculation {

    //create a 2d array to store the distance and angle
    private static double[][] distanceAngle = 
    {
        //distance, angle
        new double[] {1.22, 15},
        new double[] {1.32, 15.6},
        new double[] {1.42, 16.67},
        new double[] {1.52, 20.4},
        new double[] {1.72, 21.4},
        new double[] {1.89, 24.5},
        new double[] {1.95, 24.5},
        new double[] {2.17, 26.82},
        new double[] {2.45, 27.5},
        new double[] {2.6, 28.5},
        new double[] {2.8, 30},
        new double[] {3, 30.5},
    };

    private static double distanceConstant = 0.2;

    public static double getTargetAngle(double distance) {
        double angle = 30;
        for (int i = 0; i < distanceAngle.length; i++) {
            if (distance < distanceAngle[i][0]) {
                if (i == 0) {
                    return distanceAngle[i][1];
                }
                double maxAngle = distanceAngle[i][1];
                double minAngle = distanceAngle[i - 1][1];
                double maxDistance = distanceAngle[i][0];
                double minDistance = distanceAngle[i - 1][0];
                angle = minAngle + (distance - minDistance) * (maxAngle - minAngle) / (maxDistance - minDistance);
                return angle;
            }
        }
        return angle;
    }

    public static double getDistance(Pose2d currentPose, Pose2d targetPose) {
        return Math.sqrt(Math.pow(targetPose.getTranslation().getX() - currentPose.getTranslation().getX(), 2) + Math.pow(targetPose.getTranslation().getY() - currentPose.getTranslation().getY(), 2)); // sqrt((x2-x1)^2 + (y2-y1)^2)
    }

    public static Pose2d getGhostPosition(ChassisSpeeds speed, Pose2d currentPose) {
        Transform2d Displacement = new Transform2d(speed.vxMetersPerSecond * distanceConstant * frc.robot.subsystems.Swerve.distance, speed.vyMetersPerSecond * distanceConstant * frc.robot.subsystems.Swerve.distance, new Rotation2d(0));
        return currentPose.plus(Displacement);
    }
}
