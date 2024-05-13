// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Mechanism;

public class shooting extends Command {
  /** Creates a new shooting. */
  Mechanism mechanism;
  double shooterStartTime;

  public shooting(Mechanism m_Mechanism) {
    this.mechanism = m_Mechanism;
  }

  @Override
  public void initialize() {
    mechanism.StartShooter();
    shooterStartTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - shooterStartTime > Constants.MechanismConstants.shooterRampupTime) {
      mechanism.StartIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mechanism.StopIntake();
    mechanism.StopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - shooterStartTime > Constants.MechanismConstants.shooterTimeOutLimit) {
      return true;
    }
    return false;
  }
}
