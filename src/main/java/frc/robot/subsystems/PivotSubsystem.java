// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlignShooterCalculation;
import frc.robot.Constants;


public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */

  private final CANSparkMax leftPivotMotor;
  private final CANSparkMax rightPivotMotor;
  private final RelativeEncoder rightEncoder, leftEncoder;
  private static double targetAngle = 0;
  private SparkPIDController right_pidController, left_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private double Kff;

  public PivotSubsystem() {
    leftPivotMotor = new CANSparkMax(Constants.PivotConstants.leftPivotMotorID, CANSparkMax.MotorType.kBrushless);
    rightPivotMotor = new CANSparkMax(Constants.PivotConstants.rightPivotMotorID, CANSparkMax.MotorType.kBrushless);
    leftPivotMotor.setInverted(Constants.PivotConstants.leftPivotMotorInverted);
    rightPivotMotor.setInverted(Constants.PivotConstants.rightPivotMotorInverted);

    //set up encoder
    rightEncoder = rightPivotMotor.getEncoder();
    leftEncoder = leftPivotMotor.getEncoder();
    //encoder.setZeroOffset(Constants.PivotConstants.encoderOffset);
    rightEncoder.setPositionConversionFactor(Constants.PivotConstants.encoderPositionConversionFactor);
    leftEncoder.setPositionConversionFactor(Constants.PivotConstants.encoderPositionConversionFactor);

    rightEncoder.setPosition(Constants.PivotConstants.PositionOffset);
    leftEncoder.setPosition(Constants.PivotConstants.PositionOffset);

    //get PID controller
    right_pidController = rightPivotMotor.getPIDController();
    left_pidController = leftPivotMotor.getPIDController();
    kP = Constants.PivotConstants.kP; 
    kI = Constants.PivotConstants.kI;
    kD = Constants.PivotConstants.kD; 
    kIz = Constants.PivotConstants.kI; 
    kFF = Constants.PivotConstants.KFF; 
    kMaxOutput = Constants.PivotConstants.MaxOutput; 
    kMinOutput = Constants.PivotConstants.Minoutput;

    right_pidController.setP(kP);
    right_pidController.setI(kI);
    right_pidController.setD(kD);
    right_pidController.setIZone(kIz);
    right_pidController.setOutputRange(kMinOutput, kMaxOutput);

    left_pidController.setP(kP);
    left_pidController.setI(kI);
    left_pidController.setD(kD);
    left_pidController.setIZone(kIz);
    left_pidController.setFF(Math.cos(targetAngle * (Math.PI/180))*kFF);
    left_pidController.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("Set Rotations", targetAngle);

    //set the default angle
    targetAngle = Constants.PivotConstants.PositionOffset;
  }

  public void IncrementAngle(double increment) {
    targetAngle += increment;
  }

 public void setOutput(double output) { // only use when not using PID
    rightPivotMotor.set(output * Constants.PivotConstants.kMaxOutput);
    leftPivotMotor.set(output * Constants.PivotConstants.kMaxOutput);
  }

  public void SetAngle(double angle) {
    targetAngle = angle;
  }

  public void setPickUpAngle() {
    targetAngle = Constants.PivotConstants.pickUpAngle;
  }

  public void setUnderStageAngle() {
    targetAngle = Constants.PivotConstants.setUnderStageAngle;
  }

  public void setAmpLoadingAngle() {
    targetAngle = Constants.PivotConstants.ampAngle;
  }

  public void setSafetyAngle() {
    targetAngle = Constants.PivotConstants.safetyAngle;
  }

  public void setStartAngle() {
    targetAngle = 88;
  }

  public static void setShootingAngle(Pose2d pose) {
    Pose2d targetPose = Swerve.targetPose2d;
    double distance = AlignShooterCalculation.getDistance(pose, targetPose);
    targetAngle = AlignShooterCalculation.getTargetAngle(distance);
  }

  public double getAverageEncoderValue() {
    return (rightEncoder.getPosition() + leftEncoder.getPosition()) / 2;
  }

  public boolean isAtTarget() {
    return Math.abs(targetAngle - getAverageEncoderValue()) < Constants.PivotConstants.kToleranceDegrees;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //put encoder value to smart dashboard
    SmartDashboard.putNumber("Right Pivot encoder value", rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Pivot encoder value", leftEncoder.getPosition());

    //The moveToPosition should be called here to always run.
    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);

    // if((p != kP)) { left_pidController.setP(p); right_pidController.setP(p); kP = p; }
    // if((i != kI)) { left_pidController.setI(i); right_pidController.setI(i); kI = i; }
    // if((d != kD)) { left_pidController.setD(d); right_pidController.setD(d); kD = d; }
    // if((iz != kIz)) { left_pidController.setIZone(iz); right_pidController.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { left_pidController.setFF(ff); left_pidController.setFF(ff); kFF = ff; }
    // if((max != kMaxOutput) || (min != kMinOutput)) { 
    //   left_pidController.setOutputRange(min, max);
    //   right_pidController.setOutputRange(min, max);
    //   kMinOutput = min; kMaxOutput = max; 
    // }

    left_pidController.setFF(Math.cos(targetAngle*(180/Math.PI)) * kFF * Constants.PivotConstants.leftMultiplier);
    right_pidController.setFF(Math.cos(targetAngle*(180/Math.PI)) * kFF);
    left_pidController.setReference(targetAngle, CANSparkMax.ControlType.kPosition);
    right_pidController.setReference(targetAngle, CANSparkMax.ControlType.kPosition);

    SmartDashboard.putNumber("SetPoint", targetAngle);
    SmartDashboard.putBoolean("Pivot at AngleTarget", isAtTarget());
    //SmartDashboard.putNumber("ProcessVariable", encoder.getDistance());
  }
}
