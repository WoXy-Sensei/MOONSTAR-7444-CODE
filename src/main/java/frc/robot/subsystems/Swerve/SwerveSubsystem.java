package frc.robot.subsystems.Swerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.SwerveState;
import frc.robot.subsystems.Subsystem;
import edu.wpi.first.math.trajectory.Trajectory;

public class SwerveSubsystem extends Subsystem {

  AHRS ahrs;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;
  private Field2d field;
  private static SwerveSubsystem mInstance;

  public SwerveSubsystem() {

    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    zeroGyro();

    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, SwerveConstants.Mod0.constants),
        new SwerveModule(1, SwerveConstants.Mod1.constants),
        new SwerveModule(2, SwerveConstants.Mod2.constants),
        new SwerveModule(3, SwerveConstants.Mod3.constants)
    };

    Timer.delay(3.0);
    resetModulesToAbsolute();

    swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getYawNavX(), getModulePositions());

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry, // Robot pose supplier
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5, 0.0, 0.0), // Translation PID constants
            new PIDConstants(3.0, 0.0, 0.0), // Rotation PID constants
            4.5  , // Max module speed, in m/s
            SwerveConstants.centerToWheel,
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          return false;
        }, this);

  }

  public static SwerveSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new SwerveSubsystem();
    }
    return mInstance;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return SwerveConstants.swerveKinematics.toChassisSpeeds(getStates());
  }

  public SwerveControllerCommand getSwerveControllerCommand(Trajectory trajectory) {
    return new SwerveControllerCommand(
        trajectory,
        this::getPose,
        Constants.SwerveConstants.swerveKinematics,
        Constants.AutoConstants.xController,
        Constants.AutoConstants.yController,
        Constants.AutoConstants.thetaController,
        this::setModuleStates,
        this);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    var states = SwerveConstants.swerveKinematics.toSwerveModuleStates(robotRelativeSpeeds);
    setModuleStates(states);
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation());

    driveRobotRelative(robotRelative);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    if (translation.getX() < 0.1 && translation.getY() < 0.1 && rotation < 0.2)
      RobotState.setSwerve(SwerveState.REST);
    else
      RobotState.setSwerve(SwerveState.MOVING);

    SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYawNavX())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYawNavX(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }
  
  public void zeroGyro() {
    ahrs.zeroYaw();
  }

  public void stopModules() {
    for (SwerveModule mod : mSwerveMods) {
      mod.stopMotors();
    }
  }

  public double getHeadingNavX() {
    return Math.IEEEremainder(ahrs.getAngle(), 360) ;
  }

  public Rotation2d getYawNavX() {
    return Rotation2d.fromDegrees(getHeadingNavX());
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  @Override
  public void periodic() {

  }

  @Override
  public void writePeriodicOutputs() {

  }

  @Override
  public void stop() {

  }

  @Override
  public void outputTelemetry() {
    swerveOdometry.update(getYawNavX(), getModulePositions());
    field.setRobotPose(getPose());
    SmartDashboard.putNumber("NavxGetyaw", getYawNavX().getDegrees());
    SmartDashboard.putNumber("gyroYaw", getHeadingNavX());
    
      for (SwerveModule mod : mSwerveMods) {
        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder",mod.getCanCoder().getDegrees());
        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " AMP DRIVE",mod.getDriveOutputCurrent());
        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " AMP ROT",mod.getRotOutputCurrent());

        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " VELOCITY DRIVE",mod.getDriveVelocity());
        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " VELOCITY ROT",mod.getRotVelocity());      }

  }

  @Override
  public void reset() {
  }

}