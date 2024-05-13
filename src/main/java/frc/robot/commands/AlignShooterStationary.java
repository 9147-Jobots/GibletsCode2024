// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Routines.AlignShooterAngle;
import frc.robot.subsystems.PivotSubsystem;

public class AlignShooterStationary extends Command {
  /** Creates a new AlignShooterStationary. */
  public static boolean isFinished = false;
  private PivotSubsystem m_pivotSubsystem;
  public AlignShooterStationary(PivotSubsystem pivotSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_pivotSubsystem = pivotSubsystem;
    addRequirements(m_pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AlignShooterAngle.execute(m_pivotSubsystem);
  }

  public static void setIsFinished(boolean finished) {
    isFinished = finished;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
