package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.lib.utils.CANSparkMaxUtil;
import frc.lib.utils.CANSparkMaxUtil.Usage;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;

  private final SparkPIDController driveController;
  private final SparkPIDController angleController;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(SwerveConstants.angleContinuousCurrentLimit);
    angleMotor.setInverted(SwerveConstants.angleInvert);
    angleMotor.setIdleMode(SwerveConstants.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(SwerveConstants.angleConversionFactor);
    angleController.setP(SwerveConstants.angleKP);
    angleController.setI(SwerveConstants.angleKI);
    angleController.setD(SwerveConstants.angleKD);
    angleController.setFF(SwerveConstants.angleKFF);
    angleMotor.enableVoltageCompensation(SwerveConstants.voltageComp);
    angleMotor.burnFlash();
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(SwerveConstants.driveContinuousCurrentLimit);
    driveMotor.setInverted(SwerveConstants.driveInvert);
    driveMotor.setIdleMode(SwerveConstants.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(SwerveConstants.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(SwerveConstants.driveConversionPositionFactor);
    driveController.setP(SwerveConstants.driveKP);
    driveController.setI(SwerveConstants.driveKI);
    driveController.setD(SwerveConstants.driveKD);
    driveController.setFF(SwerveConstants.driveKFF);
    driveMotor.enableVoltageCompensation(SwerveConstants.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);

  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  public void stopMotors() {
    driveMotor.set(0);
    angleMotor.set(0);
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees((angleEncoder.getAbsolutePosition().waitForUpdate(0.1).getValue()) * 360);
  }

  public double getDriveOutputCurrent() {
    return driveMotor.getOutputCurrent();
  }

  public double getRotOutputCurrent() {
    return angleMotor.getOutputCurrent();
  }

  public double getRotVelocity() {
    return integratedAngleEncoder.getVelocity();
  }

   public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }
}
