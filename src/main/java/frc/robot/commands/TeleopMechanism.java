// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanism;

public class TeleopMechanism extends Command {
  /** Creates a new TeleopMechanism. */
  DoubleSupplier leftTriggerValue;
  DoubleSupplier rightTriggerValue;
  BooleanSupplier leftBumper;
  Mechanism s_Mechanism;
  public TeleopMechanism(Mechanism mechanism, DoubleSupplier leftTrigger, DoubleSupplier rightTrigger, BooleanSupplier leftBumper) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Mechanism = mechanism;
    leftTriggerValue = leftTrigger;
    rightTriggerValue = rightTrigger;
    this.leftBumper = leftBumper;
    addRequirements(s_Mechanism);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (leftTriggerValue.getAsDouble() > 0.4) {
      s_Mechanism.LoadAmp();
    } else if (leftBumper.getAsBoolean()) {
      s_Mechanism.StartShooter();
    } else {
      s_Mechanism.StopShooter();
    }
    if (rightTriggerValue.getAsDouble() > 0.3) {
      s_Mechanism.StartIntake();
    } else {
      s_Mechanism.StopIntake();
    }
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
