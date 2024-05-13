// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Mechanism;

public class LoadAmp extends Command {
  /** Creates a new LoadAmp. */
  Mechanism mechanism;
  double shooterStartTime;
  public LoadAmp(Mechanism m_Mechanism) {
    this.mechanism = m_Mechanism;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mechanism.StartIntake();
    mechanism.LoadAmp();
    shooterStartTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
