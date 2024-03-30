package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotState;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.Subsystem;

public class LEDsubsystem extends Subsystem {
    private static AddressableLED m_led;
    private static AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue;
    private static LEDsubsystem mInstance;
    private boolean blink;
    private Timer m_Timer;
    private double previous;

    public static LEDsubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new LEDsubsystem();
        }
        return mInstance;
    }

    /** Creates a new LEDSubsystem. */
    public LEDsubsystem() {
        m_led = new AddressableLED(LEDConstants.k_LEDPort);
        m_ledBuffer = new AddressableLEDBuffer(LEDConstants.k_BuffLength);

        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();

        m_Timer = new Timer();
        m_Timer.reset();
    }

    @Override
    public void periodic() {
        if (!RobotState.gamePieceInAmp && !RobotState.gamePieceInShooter) {
            setAll(Color.kBlack);
        } else if (RobotState.shootAmp) {
            setAll(Color.kRed);
        } else if (RobotState.robotAlignSpeaker && !RobotState.shooterMotorState && !RobotState.shooterMotorReady) {
            setAll(Color.kMagenta);
        } else if (RobotState.robotAlignSpeaker && RobotState.shooterMotorState && RobotState.shooterMotorReady) {
            blinkLED(Color.kMagenta, 0.1);
        } else if (RobotState.shooterMotorState && !RobotState.shooterMotorReady && !RobotState.gamePieceInAmp) {
            setAll(Color.kWhite);
        } else if (RobotState.shooterMotorReady && !RobotState.gamePieceInAmp && !RobotState.robotAlignSpeaker) {
            blinkLED(Color.kWhite, 0.1);
        } else if (RobotState.gamePieceInShooter) {
            blinkLED(Color.kGreen, 0.1);
        } else if (RobotState.gamePieceInAmp) {
            setAll(Color.kOrange);
        } else {
            setAll(Color.kBlack);
        }
    }

    @Override
    public void writePeriodicOutputs() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void reset() {

    }

    public void setAll(Color color) {
        for (var i = 0; i < 15; i++) {
            m_ledBuffer.setLED(i, color);
        }

        m_led.setData(m_ledBuffer);

    }

    public void setAllRGB(int R, int G, int B) {
        for (var i = 0; i < 10; i++) {
            m_ledBuffer.setRGB(i, R, G, B);
        }

        m_led.setData(m_ledBuffer);

    }

    private void rainbow() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
    }

    private void setRainbow() {
        rainbow();
        m_led.setData(m_ledBuffer);
    }

    private void blinkLED(Color color, double blinkDelay) {

        if (m_Timer.get() == 0)
            m_Timer.start();
        if (m_Timer.get() - previous > blinkDelay) {
            previous = m_Timer.get();
            blink = !blink;
        }
        if (blink) {
            setAll(color);
        } else {
            setAll(Color.kBlack);
        }
    }

}
