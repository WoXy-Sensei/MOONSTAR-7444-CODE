package frc.robot.subsystems.Amp;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.AmpConstants;
import frc.robot.subsystems.Subsystem;

public class AmpSubsystem extends Subsystem {
    private final WPI_VictorSPX ampMotor;
    private double trigger_speed;
    private final DigitalInput proximity;

    private static AmpSubsystem mInstance;

    public static AmpSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new AmpSubsystem();
        }
        return mInstance;
    }

    public AmpSubsystem() {

        ampMotor = new WPI_VictorSPX(AmpConstants.k_AmprMotor);
        ampMotor.setNeutralMode(AmpConstants.AmpTriggerNeutralMode);
        proximity = new DigitalInput(AmpConstants.k_ProximitySensor);

    }

    @Override
    public void periodic() {

    }

    @Override
    public void writePeriodicOutputs() {

        ampMotor.set(trigger_speed);
    }

    @Override
    public void stop() {
        stopTrigger();
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void reset() {

    }

    public void trigger() {
        trigger_speed = AmpConstants.k_AmpTriggerSpeed;
    }

    public void stopTrigger() {
        trigger_speed = 0;
    }

    public boolean getElevatorTriggerMotorState() {
        if (Math.abs(trigger_speed) > 0) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isObjectDetected() {
        return (proximity.get() ? false : true);
    }

    public boolean isNotObjectDetected() {
        return (proximity.get() ? true : false);
    }

}
