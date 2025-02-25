// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Mechanism;

public class IntakeNote extends Command {
  /** Creates a new IntakeSwitch. */
  Mechanism mechanism;
  XboxController xController = new XboxController(0);
  double startTime;
  boolean Stop;
  
  public IntakeNote(Mechanism m_Mechanism) {
    this.mechanism = m_Mechanism;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Swerve.UpdateVisionPose();
    mechanism.StartIntake();
    startTime = Timer.getFPGATimestamp();
    Stop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Stop == false) {
      startTime = Timer.getFPGATimestamp();
    }
    if (mechanism.AreAnyLimitSwitchesOn()) {
      Stop = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mechanism.StopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (xController.getBButton() || Timer.getFPGATimestamp() - startTime > Constants.MechanismConstants.IntakeExtraTime) {
      return true;
    }
    return false;
  }
}
