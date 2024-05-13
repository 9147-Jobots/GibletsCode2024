// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class ShooterPivotCalibration extends Command {
  /** Creates a new ShooterPivotCalibration. */
  private final PivotSubsystem s_Pivot;
  private final DoubleSupplier yAxis;
  public ShooterPivotCalibration(PivotSubsystem s_Pivot, DoubleSupplier yAxis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Pivot = s_Pivot;
    this.yAxis = yAxis;
    addRequirements(s_Pivot);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(yAxis.getAsDouble()) < 0.1) {
      return;
    }
    s_Pivot.IncrementAngle(0.5 * yAxis.getAsDouble());
  }
}
