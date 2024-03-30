package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Subsystem;

public class ClimbSubsystem extends Subsystem {
    private final WPI_VictorSPX climbMotor;
    private double climber_speed;
    private ClimbState climb_state;

    private static ClimbSubsystem mInstance;

    public static ClimbSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ClimbSubsystem();
        }
        return mInstance;
    }

    public enum ClimbState {
        NONE,
        CLIMBUP,
        CLIMBDOWN,
        STOP
    }

    public ClimbSubsystem() {
        climbMotor = new WPI_VictorSPX(ClimbConstants.k_ClimbMotor);
        climbMotor.setNeutralMode(ClimbConstants.ClimbNeutralMode);

    }

    @Override
    public void periodic() {

    }

    @Override
    public void writePeriodicOutputs() {
        climber_speed = climbStateToSpeed(climb_state);
        climbMotor.set(ControlMode.PercentOutput, climber_speed);
    }

    @Override
    public void stop() {
        stopClimb();
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Climber Speed", climber_speed);
    }

    @Override
    public void reset() {

    }

    public double climbStateToSpeed(ClimbState mode) {
        switch (mode) {
            case CLIMBDOWN:
                return ClimbConstants.k_ClimbDownSpeed;
            case CLIMBUP:
                return ClimbConstants.k_ClimbUpSpeed;
            case STOP:
                return 0.0;
            default:
                return 0.0;
        }
    }

    public void climbUp() {
        climb_state = ClimbState.CLIMBUP;
    }

    public void climbDown() {
        climb_state = ClimbState.CLIMBDOWN;
    }

    public void stopClimb() {
        climb_state = ClimbState.STOP;
    }

    public boolean getClimbMotorState() {
        if (Math.abs(climber_speed) > 0.1) {
            return true;
        } else {
            return false;
        }
    }

}
