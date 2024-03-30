package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Subsystem;

public class IntakeSubsystem extends Subsystem {
    private final CANSparkMax m_IntakeMotor;
    private double intake_speed;
    private IntakeState intake_state;


    private static IntakeSubsystem mInstance;

    public static IntakeSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new IntakeSubsystem();
        }   
        return mInstance;
    }

    public enum IntakeState {
        NONE,
        INTAKE,
        INTAKE_TRIG,
        INTAKE_REVERSE,
        STOP
    }

    public IntakeSubsystem() {
        m_IntakeMotor = new CANSparkMax(IntakeConstants.k_IntakeMotor, MotorType.kBrushless);
        m_IntakeMotor.setIdleMode(IdleMode.kBrake);

        m_IntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
        m_IntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 100);
        m_IntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 100);

        m_IntakeMotor.burnFlash();

    }

    @Override
    public void periodic() {
        intake_speed = intakeStateToSpeed(intake_state);
    }

    @Override
    public void writePeriodicOutputs() {
        m_IntakeMotor.set(intake_speed);
    }

    @Override
    public void stop() {
        stopIntake();
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Intake Speed", intake_speed);
    }

    @Override
    public void reset() {

    }

    public double intakeStateToSpeed(IntakeState mode) {
        switch (mode) {
            case INTAKE:
                return IntakeConstants.k_intakeSpeed;
            case INTAKE_TRIG:
                return IntakeConstants.k_intakeTrigSpeed;
            case INTAKE_REVERSE:
                return IntakeConstants.k_reverseIntakeSpeed;
            case STOP:
                return 0.0;
            default:
                return 0.0;
        }
    }

    public void intake() {
        intake_state = IntakeState.INTAKE;
    }

    public void intake_trig() {
        intake_state = IntakeState.INTAKE_TRIG;
    }

    public void intake_reverse() {
        intake_state = IntakeState.INTAKE_REVERSE;
    }

    public void stopIntake() {
        intake_state = IntakeState.STOP;
    }

    public boolean getIntakeMotorState() {

        if (Math.abs(intake_speed) > 0.1) {
            return true;
        } else {
            return false;
        }
    }

}
