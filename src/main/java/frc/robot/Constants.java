package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import frc.lib.config.SwerveModuleConstants;
import frc.lib.utils.LinearInterpolator;

public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAsisstantControllerPort = 1;

  }

  public static class LimelightConstants {
    public static final double target_heightCm = 132.0;
    public static final double limelightMountAngleDegrees = 30;
    public static final double limelightLensHeightCm = 38.5;
  }

  public static final class SwerveConstants {
    public static final double stickDeadband = 0.1;
    public static final boolean invertGyro = false; 

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(28.74); 
    public static final double wheelBase = Units.inchesToMeters(30.31);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final double centerToWheel = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

    public static final double driveGearRatio = (6.75 / 1.0);
    public static final double angleGearRatio = (21.42 / 1.0);

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 50;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false; // Always ensure canCoder is CCW+ CW-

    /* Angle Motor PID Values */
    public static final double angleKP = 0.005;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.005;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.0; 
    public static final double driveKA = 0.2; 

    /* Drive Motor Conversion Factors */
    public static final double driveReductionMK4I = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double steerReductionMK4I = (14.0 / 50.0) * (10.0 / 60.0);
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5;
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kCoast;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Module Specific Constants */

    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 6;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(21.53);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(274.03); 
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(339.43); 
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(146.3); 
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 6;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3; 
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.3;
    public static final double kPYController = 1.3;
    public static final double kPThetaController = 3.5;
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0.02);
    public static final PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0.02);
    public static final ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  }

  public static final class IntakeConstants {
    public static final int k_IntakeMotor = 17;

    public static final NeutralMode IntakeNeutralMode = NeutralMode.Brake;

    public static final double k_intakeSpeed = -0.95;
    public static final double k_reverseIntakeSpeed = 0.95;
    public static final double k_intakeTrigSpeed = -0.3;

  }

  public static final class ShooterConstants {
    public static final int k_ShooterMotor1 = 13;
    public static final int k_ShooterMotor2 = 14;
    public static final int k_ShooterTriggerMotor = 20;
    public static final int k_ShooterPivotMotor = 15;

    public static final IdleMode ShooterIdleMode = IdleMode.kBrake;
    public static final NeutralMode ShooterPivotNeutralMode = NeutralMode.Brake;

    public static final double k_ShooterSpeed = 1;
    public static final double k_ShootAmpSpeed = 0.5;
    public static final double k_TriggerSpeed = -1;
    public static final double k_TriggerReverseSpeed = 0.25;
    

    public static final double k_ShooterOffset = 327;
    public static final double k_ShooterStartAngle = 100;

    public static final double[][] SHOOTER_ANGLE_ARRAY = {
        { 80,  210}, 
        { 130 , 145}, 
        { 180, 108}, 
        { 230 , 70}, 
        { 280 , 55}, 
        { 330 , 45}, 
    };

    public static final LinearInterpolator SHOOTER_INTERPOLATOR = new LinearInterpolator(SHOOTER_ANGLE_ARRAY);

    public static final int k_Encoder = 0;
    public static final int k_ProximitySensor = 1;

    public static final int ShooterContinuousCurrentLimit = 50;
    public static final int TriggerContinuousCurrentLimit = 50;

  
  }

  public static final class ClimbConstants {
    public static final int k_ClimbMotor = 16;

    public static final NeutralMode ClimbNeutralMode = NeutralMode.Brake;

    public static final double k_ClimbUpSpeed = 1;
    public static final double k_ClimbDownSpeed = -1;

  }

  public static final class AmpConstants {
    public static final int k_AmprMotor = 18;

    public static final NeutralMode AmpTriggerNeutralMode = NeutralMode.Brake;

    public static final double k_AmpTriggerSpeed = -0.60; 

    public static final int k_ProximitySensor = 2;
  }

  public static final class LEDConstants {
    public static final int k_LEDPort = 9;
    public static final int k_BuffLength = 15;
  }

}