// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.VisionSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //start data logging
    DataLogManager.start();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    DataLog datalog = DataLogManager.getLog();

    //this logs control data
    DriverStation.startDataLog(datalog);

    //define variables for data logging
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    //check if the vision pose is accurate
    VisionSubsystem.updatePosition();
    Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(m_robotContainer.getAutoName());
    EstimatedRobotPose EstimatePose = VisionSubsystem.GetFieldPose();
    //null check
    if (EstimatePose == null) {
      Lights.red();
      return;
    }
    if (EstimatePose.equals(null) || startingPose == null) {
      Lights.red();
      return;
    }
    Pose2d visionPose = EstimatePose.estimatedPose.toPose2d();
    //compare the vision pose to the starting pose x, y and rotation
    //if the difference is greater than 0.1, then the vision pose is not accurate
    if (Math.abs(visionPose.getTranslation().getX() - startingPose.getX()) > 0.2) {
      Lights.red();
      return;
    }
    if (Math.abs(visionPose.getTranslation().getY() - startingPose.getY()) > 0.2) {
      Lights.red();
      return;
    }
    if (Math.abs(visionPose.getRotation().getDegrees() - startingPose.getRotation().getDegrees()) > 3) {
      Lights.red();
      return;
    }
    // SmartDashboard.putNumber("X error", visionPose.getTranslation().getX() - startingPose.getX());
    // SmartDashboard.putNumber("Y error", visionPose.getTranslation().getY() - startingPose.getY());
    // SmartDashboard.putNumber("Z position", EstimatePose.estimatedPose.getZ());
    // SmartDashboard.putNumber("theta error", visionPose.getRotation().getDegrees() - startingPose.getRotation().getDegrees());
    Lights.green();
    double x = Swerve.targetPose2d.getTranslation().getX();
    double y = Swerve.targetPose2d.getTranslation().getY();
    double currentX = Swerve.getPose().getX();
    double currentY = Swerve.getPose().getY();
    SmartDashboard.putNumber("Target align angle", -Math.toDegrees(Math.atan2(currentY - y, x - currentX)));
    SmartDashboard.putNumber("Current Rotation", Swerve.getPoseYaw().getDegrees());
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
