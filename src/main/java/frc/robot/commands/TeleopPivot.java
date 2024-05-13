// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Routines.AlignShooterAngle;
import frc.robot.subsystems.PivotSubsystem;

public class TeleopPivot extends Command {
  /** Creates a new TeleopPivot. */
  DoubleSupplier s_pivotDoubleSupplier;
  BooleanSupplier s_autoAim;
  PivotSubsystem s_pivotSubsystem;
  public TeleopPivot(PivotSubsystem pivotSubsystem, DoubleSupplier pivotincrement, BooleanSupplier autoAim) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_pivotSubsystem = pivotSubsystem;
    s_autoAim = autoAim;
    s_pivotDoubleSupplier = pivotincrement;
    addRequirements(s_pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_autoAim.getAsBoolean()) {
      AlignShooterAngle.execute(s_pivotSubsystem);
      return;
    }
    double increment = s_pivotDoubleSupplier.getAsDouble();
    if (Math.abs(increment) < 0.5) {
      return;
    }
    s_pivotSubsystem.IncrementAngle(increment * 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
