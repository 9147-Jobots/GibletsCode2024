// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SystemConstants.GyroConstants;

public class GyroSubsystem extends SubsystemBase {
  /** Creates a new Gyro_subsystem. */

  private final static AHRS Gyro = new AHRS();

  public GyroSubsystem() {}

  public Rotation2d GetRotation2d() {
    return Gyro.getRotation2d();
  }

  public static double get_yaw() {
    return Gyro.getYaw() - GyroConstants.YAW_OFFSET;
  }

  public double get_pitch() {
    return Gyro.getPitch() - GyroConstants.PITCH_OFFSET;
  }

  public double get_roll() {
    return Gyro.getRoll() - GyroConstants.ROLL_OFFSET;
  }

  public static void reset_yaw() {
    Gyro.reset();
  }
  
  public static double Accel_x() {
    return Gyro.getRawAccelX();
  }

  public static double Accel_y() {
    return Gyro.getRawAccelY();
  }

  public static double Accel_z() {
    return Gyro.getRawAccelZ();
  }

  public static double Temp() {
    return Gyro.getTempC();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
