// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class TestPivot extends Command {
  /** Creates a new TestPivot. */
  private final PivotSubsystem s_Pivot;
  private final DoubleSupplier yAxis;
  public TestPivot(PivotSubsystem s_Pivot, DoubleSupplier yAxis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Pivot = s_Pivot;
    this.yAxis = yAxis;
    addRequirements(s_Pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Pivot.setOutput(yAxis.getAsDouble());
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
