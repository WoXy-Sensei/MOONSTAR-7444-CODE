package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.vision.LimeLight;
import frc.robot.Helpers;
import frc.robot.RobotState;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Subsystem;

public class ShooterSubsystem extends Subsystem {
    private final CANSparkMax shooterMotor1, shooterMotor2;
    private final WPI_VictorSPX pivotMotor;
    private final CANSparkMax triggerMotor;
    private double shooter_speed;
    private double pivot_speed;
    private double trigger_speed;
    private double pivot_angle;
    private final DutyCycleEncoder encoder;
    private double output_pivot_speed;
    private DigitalInput proximity;
    private final PIDController pivotPID = new PIDController(0.031, 0, 0);
    private static ShooterSubsystem mInstance;
    private RelativeEncoder shooterEncoder1, shooterEncoder2;

    LimeLight limeLight = new LimeLight();

    public static ShooterSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ShooterSubsystem();
        }
        return mInstance;
    }

    public ShooterSubsystem() {
        pivotPID.setTolerance(3);

        shooterMotor1 = new CANSparkMax(ShooterConstants.k_ShooterMotor1, MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(ShooterConstants.k_ShooterMotor2, MotorType.kBrushless);
        pivotMotor = new WPI_VictorSPX(ShooterConstants.k_ShooterPivotMotor);
        triggerMotor = new CANSparkMax(ShooterConstants.k_ShooterTriggerMotor, MotorType.kBrushless);
        encoder = new DutyCycleEncoder(ShooterConstants.k_Encoder);
        encoder.setDistancePerRotation(500);
        pivotMotor.setNeutralMode(ShooterConstants.ShooterPivotNeutralMode);

        shooterMotor1.restoreFactoryDefaults();
        shooterMotor2.restoreFactoryDefaults();

        shooterMotor1.follow(shooterMotor2);

        shooterMotor1.setIdleMode(ShooterConstants.ShooterIdleMode);
        shooterMotor2.setIdleMode(ShooterConstants.ShooterIdleMode);
        triggerMotor.setIdleMode(ShooterConstants.ShooterIdleMode);

        shooterMotor1.setSmartCurrentLimit(ShooterConstants.ShooterContinuousCurrentLimit);
        shooterMotor2.setSmartCurrentLimit(ShooterConstants.ShooterContinuousCurrentLimit);
        triggerMotor.setSmartCurrentLimit(ShooterConstants.TriggerContinuousCurrentLimit);

        triggerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
        triggerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 100);
        triggerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 100);

        shooterMotor1.burnFlash();
        shooterMotor2.burnFlash();
        triggerMotor.burnFlash();

        proximity = new DigitalInput(ShooterConstants.k_ProximitySensor);

        shooterEncoder1 = shooterMotor1.getEncoder();
        shooterEncoder2 = shooterMotor2.getEncoder();

        pivot_angle = ShooterConstants.k_ShooterStartAngle;
    }

    @Override
    public void periodic() {
        output_pivot_speed = pivotPID.calculate(getAngle(), pivot_angle);

        if (limeLight.getdegVerticalToTarget() != 0 && RobotState.isTeleop() && RobotState.pivotReady
                && !RobotState.limelightDeactive) {
            SmartDashboard.putNumber("apriltag distance", limeLight.getDistanceLLToGoal());
            SmartDashboard.putNumber("Interpolated Shooter Angle",
                    ShooterConstants.SHOOTER_INTERPOLATOR.getInterpolatedValue(limeLight.getDistanceLLToGoal()));
            pivot_angle = ShooterConstants.SHOOTER_INTERPOLATOR.getInterpolatedValue(limeLight.getDistanceLLToGoal());
        }

        if (limeLight.getDistanceLLToGoal() > 350 || !limeLight.getIsTargetFound()) {
            RobotState.pivotReady = false;
        } else {
            RobotState.pivotReady = true;
        }

        if (isObjectDetected()) {
            RobotState.gamePieceInShooter = true;
        } else {
            RobotState.gamePieceInShooter = false;
        }

        if (getShooteMotorState()) {
            RobotState.shooterMotorState = true;
        } else {
            RobotState.shooterMotorState = false;
        }

        if (Helpers.inRange(getShooter1Velocity(), 5000, 6000)) {
            RobotState.shooterMotorReady = true;
        } else {
            RobotState.shooterMotorReady = false;
        }

        if (Helpers.inRange(limeLight.getTargetPoseInRobotPoseRZ(), -5 , 5) && Helpers.inRange(limeLight.getdegRotationToTarget(), -6 , 6) && limeLight.getIsTargetFound()) {
            RobotState.robotAlignSpeaker = true;
        } else {
            RobotState.robotAlignSpeaker = false;
        }
    }

    @Override
    public void writePeriodicOutputs() {
        pivotMotor.set(output_pivot_speed);
        shooterMotor2.set(shooter_speed);
        triggerMotor.set(trigger_speed);

    }

    @Override
    public void stop() {
        pivot_speed = 0.0;
        shooter_speed = 0.0;
        trigger_speed = 0.0;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Pivot Angle Varible", pivot_angle);
        SmartDashboard.putNumber("Pivot Angle Encoder", getAngle());
        SmartDashboard.putNumber("Pivot Distance", encoder.getDistance());
        SmartDashboard.putNumber("Pivot Speed", pivot_speed);
        SmartDashboard.putNumber("Pivot PID", output_pivot_speed);
        SmartDashboard.putNumber("Shooter Speed", shooter_speed);
        SmartDashboard.putNumber("Shooter velocity 1", getShooter1Velocity());
        SmartDashboard.putNumber("Shooter velocity 2", getShooter2Velocity());
        SmartDashboard.putNumber("Shooter Current 1", getShooter1Current());
        SmartDashboard.putNumber("Shooter Current 2", getShooter2Current());
    }

    @Override
    public void reset() {

    }

    public void shoot() {
        shooter_speed = ShooterConstants.k_ShooterSpeed;
    }

    public void shootAmp() {
        shooter_speed = ShooterConstants.k_ShootAmpSpeed;
    }

    public void trigger() {
        trigger_speed = ShooterConstants.k_TriggerSpeed;
    }

    public void triggerReverse() {
        trigger_speed = ShooterConstants.k_TriggerReverseSpeed;
    }

    public void stopShoot() {
        shooter_speed = 0;
    }

    public void stopTrigger() {
        trigger_speed = 0;
    }

    public void setPivotAngle(double angle) {
        pivot_angle = angle;
    }

    public void setPivotSpeed(double speed) {
        pivot_speed = speed;
    }

    public void pivotUp() {
        pivot_speed = 1;
    }

    public void pivotDown() {
        pivot_speed = -1;
    }

    public void setTriggerSpeed(double speed) {
        trigger_speed = speed;
    }

    public void setShooterSpeed(double speed) {
        shooter_speed = speed;
    }

    public boolean getShooteMotorState() {
        if (Math.abs(shooter_speed) > 0) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isShooterNotRunning() {
        if (Math.abs(shooter_speed) > 0) {
            return false;
        } else {
            return true;
        }
    }

    public boolean getTriggerMotorState() {
        if (Math.abs(trigger_speed) > 0) {
            return true;
        } else {
            return false;
        }
    }

    public boolean getPivotMotorState() {
        if (Math.abs(pivot_speed) > 0) {
            return true;
        } else {
            return false;
        }
    }

    public double getAngle() {
        double angle = encoder.getDistance() - ShooterConstants.k_ShooterOffset;
        return angle;
    }

    public boolean isObjectDetected() {
        return (proximity.get() ? false : true);
    }

    public boolean isNotObjectDetected() {
        return (proximity.get() ? true : false);
    }

    public double getShooter1Velocity() {
        return shooterEncoder1.getVelocity();
    }

    public double getShooter2Velocity() {
        return shooterEncoder2.getVelocity();
    }

    public double getShooter1Current() {
        return shooterMotor1.getOutputCurrent();
    }

    public double getShooter2Current() {
        return shooterMotor2.getOutputCurrent();
    }

    public boolean pivotAtSetpoint() {
        return pivotPID.atSetpoint();
    }

}
