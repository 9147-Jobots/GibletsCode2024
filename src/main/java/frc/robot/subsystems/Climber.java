// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkMax M_Motor;
  public Climber() {
    M_Motor = new CANSparkMax(Constants.MechanismConstants.climberID, MotorType.kBrushless);
  }

  public void StartClimber() {
    M_Motor.set(1);
  }

  public void ReverseClimber() {
    M_Motor.set(-1);
  }

  public void StopClimber() {
    M_Motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
