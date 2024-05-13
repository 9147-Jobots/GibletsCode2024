// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Mechanism;
import frc.robot.subsystems.PivotSubsystem;

public class PickupNote extends Command {
  /** Creates a new PickupNote. */
  private final Mechanism mechanism;
  private final PivotSubsystem pivotSubsystem;
  private final BooleanSupplier manualSwitchOff;
  double startTime;
  public PickupNote(Mechanism mechanism, PivotSubsystem pivotSubsystem, BooleanSupplier manualSwitchOff) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mechanism = mechanism;
    this.pivotSubsystem = pivotSubsystem;
    this.manualSwitchOff = manualSwitchOff;
    addRequirements(mechanism, pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    mechanism.StartIntake();
    pivotSubsystem.setPickUpAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mechanism.StopIntake();
    pivotSubsystem.setUnderStageAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - startTime > Constants.MechanismConstants.IntakeTimeoutLimit) {
      return true;
    }
    if (mechanism.AreAnyLimitSwitchesPersistantlyOn()) {
      return true;
    }
    return false;
  }
}
