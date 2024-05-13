// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SystemConstants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new Limelight. */

  private final PhotonCamera camera; //creating a new photon camera
  private final PhotonCamera pilotCam;
  private final Transform3d cameraToRobot; //creating a new transform3d
  public static PhotonPoseEstimator poseEstimator; //creating a new photon pose estimator
  public static PhotonPipelineResult lastResult; //creating a new photon pipeline result
  private static EstimatedRobotPose pose; //creating a new estimated robot pose
  private AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); //creating a new apriltag field layout


  public VisionSubsystem() {

    //creating a new photon camera
    camera = new PhotonCamera("photonvision");
    pilotCam = new PhotonCamera("pilotCam");
    //updating the camera
    lastResult = camera.getLatestResult();

    //creating a new transform3d
    cameraToRobot = new Transform3d(
      new Translation3d(
          VisionConstants.Transform3dConstants.x, 
          VisionConstants.Transform3dConstants.y, 
          VisionConstants.Transform3dConstants.z), 
      new Rotation3d(
          VisionConstants.Transform3dConstants.pitch, 
          VisionConstants.Transform3dConstants.roll, 
          VisionConstants.Transform3dConstants.yaw
      ));

      //creating a new photon pose estimator
      poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, cameraToRobot);
      pose = poseEstimator.update(lastResult).orElse(null);

  }

  public static EstimatedRobotPose GetFieldPose() {
    return pose;
  }

  public static double getPoseAmbiguity() {
    if (lastResult == null) {
      return -1;
    }
    if (lastResult.getTargets().isEmpty()) {
      return -1;
    }
    return lastResult.getBestTarget().getPoseAmbiguity();
  }

  public static void updatePosition() {
    pose = poseEstimator.update(lastResult).orElse(null);
  }
  
  @Override
  public void periodic() {
    // Instantiate Photon Camera with cam name
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      var target = result.getBestTarget();
      var camToTarget = target.getBestCameraToTarget();
      // SmartDashboard.putBoolean("Has targets", true);
      // SmartDashboard.putNumber("x Distance to tag", camToTarget.getX());
      // SmartDashboard.putNumber("y Distance to tag", camToTarget.getY());
      // SmartDashboard.putNumber("Angle to tag", camToTarget.getRotation().getAngle());
    } else {
      SmartDashboard.putBoolean("Has targets", false);
    }

    //update pose to Smartdashboard if result isn't null
    pose = poseEstimator.update(lastResult).orElse(null);
    if (pose != null) {
      SmartDashboard.putNumber("Pose : X", pose.estimatedPose.getTranslation().getX());
      SmartDashboard.putNumber("Pose : Y", pose.estimatedPose.getTranslation().getY());
      SmartDashboard.putNumber("Pose : Z", pose.estimatedPose.getTranslation().getZ());
      SmartDashboard.putNumber("Pose : Rotation", pose.estimatedPose.getRotation().getAngle());
    }

    //updating the camera
    lastResult = camera.getLatestResult();
  }
}
