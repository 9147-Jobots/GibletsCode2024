// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanism;

public class TeleopRevShooter extends Command {
  /** Creates a new TeleopRevShooter. */
  Mechanism s_Mechanism;
  BooleanSupplier KeepReving;
  public TeleopRevShooter(Mechanism mechanism, BooleanSupplier LBumper) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Mechanism = mechanism;
    KeepReving = LBumper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Mechanism.StartShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Mechanism.StopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return KeepReving.getAsBoolean();
  }
}
