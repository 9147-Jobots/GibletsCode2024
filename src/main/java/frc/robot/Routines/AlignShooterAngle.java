// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.AlignShooterCalculation;
import frc.robot.SystemConstants.FieldConstants;
import frc.robot.subsystems.PivotSubsystem;

/** Add your docs here. */
public class AlignShooterAngle {
    public static void execute(PivotSubsystem m_pivotSubsystem) {
        //Align the shooter to the target
        Pose2d currentPose = Swerve.getPose();
        Pose2d targetPose;
        //check Alliance
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        //if blue, set target to red
        targetPose = FieldConstants.BlueAlliance.ScoringPose;
        } else {
        //if red, set target to blue
        targetPose = FieldConstants.RedAlliance.ScoringPose;
        }
        double distance = AlignShooterCalculation.getDistance(currentPose, targetPose);
        double angle = AlignShooterCalculation.getTargetAngle(distance);
        m_pivotSubsystem.SetAngle(angle);
    }
}
