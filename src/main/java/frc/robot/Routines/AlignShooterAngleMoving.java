// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.AlignShooterCalculation;
import frc.robot.subsystems.PivotSubsystem;

/** Add your docs here. */
public class AlignShooterAngleMoving {
    public static void execute(PivotSubsystem m_pivotSubsystem) {
        //Align the shooter to the target
        Pose2d currentPose = Swerve.getPose();
        ChassisSpeeds currentSpeed = Swerve.getChassisSpeeds();
        Pose2d ghostPose2d = AlignShooterCalculation.getGhostPosition(currentSpeed, currentPose);
        Pose2d targetPose = Swerve.getTargetPose2d();
        double distance = AlignShooterCalculation.getDistance(ghostPose2d, targetPose);
        double angle = AlignShooterCalculation.getTargetAngle(distance);
        m_pivotSubsystem.SetAngle(angle);
    }
}
