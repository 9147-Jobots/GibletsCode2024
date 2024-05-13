package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
//Change most of these constants
public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        //public static final int pigeonID = 1;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.687; //TODO: This must be tuned to specific robot
        public static final double wheelBase = 0.82; //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.25;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 10; //TODO: This must be tuned to specific robot
        public static final double driveKI = 1;
        public static final double driveKD = 0;
        public static final double driveKF = 2;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 10; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 7; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        ///* MotorID Adjustments if a motor fails */
        //public static Boolean ThreeWheelMode = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-54.58);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1  or Slave Wheel if any modules fail*/
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-145.81);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-185.62);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-221.92);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class MechanismConstants { //TODO: The below constants are copied from inatke and shooter subsystems, might need to be adjusted
        public static final int intakeMotorID = 13;
        public static final int shooterTopMotorID = 14;
        public static final int shooterBottomMotorID = 15;
        public static final int climberID = 18;
        public static final boolean intakeMotorInverted = false;
        public static final boolean shooterTopMotorInverted = true;
        public static final boolean shooterBottomMotorInverted = true;

        public static final double intakeMotorSpeed = 0.7;
        public static final double shooterMotorSpeed = 0.7;
        public static final double shooterLoadAmpSpeed = 0.15;
        //for the limit switch
        public static final int limitSwitchID1 = 0;
        public static final int limitSwitchID2 = 1;
        public static final int limitSwitchID3 = 2;
        public static final double persistantOnTime = 0.03;

        //for shooter command
        public static final double shooterTimeOutLimit = 3;
        public static final double shooterRampupTime = 2;
        public static final double IntakeTimeoutLimit = 4;
        public static final double IntakeExtraTime = 0.00;
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final String auto1 = "3 note close source start";
        public static final String auto2 = "Middle Note far source start";
        public static final String auto3 = "3 Note Far source start";
        public static final String auto4 = "";
        public static final String auto5 = "";
        public static final String auto6 = "";
        public static final String auto7 = "";
        public static final String auto8 = "";

    }

    public static final class DriveConstants {
        //PID constants for autoaligning to a target angle
        // to be tuned later when testing
        public static double kP = 0.03;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 0.0;

        public static double kToleranceDegrees = 2.0f;
    }

    public static final class PivotConstants {
        public static final int leftPivotMotorID = 16;
        public static final int rightPivotMotorID = 17;
        public static final boolean leftPivotMotorInverted = true;
        public static final boolean rightPivotMotorInverted = false;
        public static final double PositionOffset = 88;

        //encoder Offset
        public static final double encoderOffset = 0; // needs to be set

        //encoder Position ConversionFactor
        public static final double encoderPositionConversionFactor = 0.75; // needs to be set

        //PID Constants
        public static final double kP = 0.045;
        public static final double kI = 5e-4;
        public static final double kD = 0.0;
        public static final double KIz = 0;
        public static final double KFF = 1e-4;
        public static final double MaxOutput = 0.6;
        public static final double Minoutput = -0.6;

        //LeftSide multiplier
        public static final double leftMultiplier = 1.1;

        //Pivot manual control max output
        public static final double kMaxOutput = 0.6;

       //Pivot Tolerance
        public static final double kToleranceDegrees = 1.0f;
        

        //PresetAngles
        public static double pickUpAngle = -8;
        public static double setUnderStageAngle = 14;
        public static double safetyAngle = 62;
        public static double ampAngle = 100;
    }
}
