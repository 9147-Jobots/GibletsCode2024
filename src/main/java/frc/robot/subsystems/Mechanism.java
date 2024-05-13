// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Mechanism extends SubsystemBase {
  private CANSparkMax intakeMotor;
  private CANSparkMax shooterTopMotor;
  private CANSparkMax shooterBottomMotor;
  private RelativeEncoder shooterTopEncoder;
  private RelativeEncoder shooterBottomEncoder;
  private DigitalInput limitSwitch1, limitSwitch2, limitSwitch3;
  private double startTime;
  private boolean switchActivated;
  private int switchCount;

  /** Creates a new Mechanism. */
  public Mechanism() {
    //for the intake
    intakeMotor = new CANSparkMax(MechanismConstants.intakeMotorID, MotorType.kBrushed);
    intakeMotor.setInverted(MechanismConstants.intakeMotorInverted);

    //for the shooter
    shooterTopMotor = new CANSparkMax( MechanismConstants.shooterTopMotorID, MotorType.kBrushless );
    shooterBottomMotor = new CANSparkMax( MechanismConstants.shooterBottomMotorID, MotorType.kBrushless );
    shooterTopEncoder = shooterTopMotor.getEncoder();
    shooterBottomEncoder = shooterBottomMotor.getEncoder();
    shooterTopMotor.setInverted(MechanismConstants.shooterTopMotorInverted);
    shooterBottomMotor.setInverted(MechanismConstants.shooterBottomMotorInverted);

    //for the limitswitch
    limitSwitch1 = new DigitalInput(MechanismConstants.limitSwitchID1);
    limitSwitch2 = new DigitalInput(MechanismConstants.limitSwitchID2);
    limitSwitch3 = new DigitalInput(MechanismConstants.limitSwitchID3);

    switchActivated = false;
  }

  public void StartIntake() {
    intakeMotor.set(MechanismConstants.intakeMotorSpeed);
  }

  public void ReverseIntake() {
    intakeMotor.set(-MechanismConstants.intakeMotorSpeed);
  }

  public void StopIntake() {
    intakeMotor.stopMotor();
  }

  public void StartShooter() {
    shooterTopMotor.set(MechanismConstants.shooterMotorSpeed);
    shooterBottomMotor.set(MechanismConstants.shooterMotorSpeed);
  }

  public void LoadAmp() {
    shooterTopMotor.set(MechanismConstants.shooterLoadAmpSpeed);
    shooterBottomMotor.set(MechanismConstants.shooterLoadAmpSpeed);
  }

  public void ReverseShooter() {
    shooterTopMotor.set(-MechanismConstants.shooterLoadAmpSpeed);
    shooterBottomMotor.set(-MechanismConstants.shooterLoadAmpSpeed);

  }
    
  public void StopShooter() {
      shooterTopMotor.stopMotor();
      shooterBottomMotor.stopMotor();
  }

  public boolean IsLimitSwitchOn(DigitalInput limitSwitch) {
    return !limitSwitch.get();
  }

  public boolean AreAnyLimitSwitchesOn() {
    SmartDashboard.putBoolean("Any limit switch on", IsLimitSwitchOn(limitSwitch1) || IsLimitSwitchOn(limitSwitch2) || IsLimitSwitchOn(limitSwitch3));
    return IsLimitSwitchOn(limitSwitch1) || IsLimitSwitchOn(limitSwitch2) || IsLimitSwitchOn(limitSwitch3);
  }

  public boolean AreAllLimitSwitchesOn() {
    return IsLimitSwitchOn(limitSwitch1) && IsLimitSwitchOn(limitSwitch2) && IsLimitSwitchOn(limitSwitch3);
  }

  public boolean IsLimitSwitchOff() {
    return limitSwitch1.get();
  }

  public double getShooterTopEncoder() {
    return shooterTopEncoder.getPosition();
  }

  public double getShooterBottomEncoder() {
    return shooterBottomEncoder.getPosition();
  }

  private void startTimingSwitch() {
    startTime = Timer.getFPGATimestamp();
    switchActivated = true;
  }

  public boolean AreAnyLimitSwitchesPersistantlyOn() {
    if (AreAnyLimitSwitchesOn()) {
      switchCount += 1;
      if (switchCount >= 2) {
        return true;
      }
      return false;
    } else {
      switchCount = 0;
      return false;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake limit switch", AreAnyLimitSwitchesOn());
    SmartDashboard.putBoolean("Intake limit switch persistant", AreAnyLimitSwitchesPersistantlyOn());
  }
}
