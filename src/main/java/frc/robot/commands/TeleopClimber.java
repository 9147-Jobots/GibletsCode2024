// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class TeleopClimber extends Command {
  /** Creates a new TeleopClimber. */
  private Climber m_Climber;
  private XboxController controller;
  public TeleopClimber(Climber climber, XboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Climber = climber;
    controller = joystick;
    addRequirements(m_Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controller.getPOV() == 0) {
      m_Climber.StartClimber();
    } else
    if (controller.getPOV() == 180) {
      m_Climber.ReverseClimber();
    } else {
      m_Climber.StopClimber();
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
