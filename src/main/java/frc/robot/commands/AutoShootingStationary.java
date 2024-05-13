// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Routines.AlignShooterAngle;
import frc.robot.subsystems.Mechanism;
import frc.robot.subsystems.PivotSubsystem;

public class AutoShootingStationary extends Command {
  /** Creates a new AutoShootingStationary. */
  Mechanism s_Mechanism;
  PivotSubsystem s_PivotSubsystem;
  double startTime;
  public AutoShootingStationary(Mechanism mechanism, PivotSubsystem pivotSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Mechanism = mechanism;
    s_PivotSubsystem = pivotSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //attempt to update pose with vision
    Swerve.UpdateVisionPose();
    s_Mechanism.StartShooter();
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AlignShooterAngle.execute(s_PivotSubsystem);
    if (Timer.getFPGATimestamp() - startTime > Constants.MechanismConstants.shooterRampupTime) {
      s_Mechanism.StartIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Mechanism.StopShooter();
    s_Mechanism.StopIntake();
    s_PivotSubsystem.setPickUpAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - startTime > Constants.MechanismConstants.shooterTimeOutLimit) {
      return true;
    }
    return false;
  }
}
