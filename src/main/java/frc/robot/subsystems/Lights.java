package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Relay;
import com.ctre.phoenix.CANifier;

public class Lights implements Subsystem {

    public static CANifier canifier;
    public static Relay relay1;
    public String vision;
    public final static CANifier.PWMChannel kMotorControllerCh = CANifier.PWMChannel.PWMChannel2;
    public Turret m_turret;

    public Lights(Turret turret) {
        setDefaultCommand(new ChangeLights(this));
        m_turret = turret;

        canifier = new CANifier(0);
        relay1 = new Relay(1);
    }

    public void onStart() {
        canifier.enablePWMOutput(kMotorControllerCh.value, true);
        relay1.set(Relay.Value.kOn);
        canifier.setLEDOutput(40, CANifier.LEDChannel.LEDChannelA);
        canifier.setLEDOutput(250, CANifier.LEDChannel.LEDChannelB);
        canifier.setLEDOutput(40, CANifier.LEDChannel.LEDChannelC);

    }

    public void onStop() {
        canifier.enablePWMOutput(kMotorControllerCh.value, false);
        relay1.set(Relay.Value.kOff);
    }

    public void onLoop() {
    }

    public boolean isDone() {
        return false;
    }

    public void changeLight() {
        vision = m_turret.getVisionString();

        if (vision.equals("locked")) {

            canifier.setLEDOutput(242, CANifier.LEDChannel.LEDChannelA);
            canifier.setLEDOutput(170, CANifier.LEDChannel.LEDChannelB);
            canifier.setLEDOutput(53, CANifier.LEDChannel.LEDChannelC);
        } else {
            if (BallSensor.getOutput() == 1) {
                canifier.setLEDOutput(250, CANifier.LEDChannel.LEDChannelA);
                canifier.setLEDOutput(250, CANifier.LEDChannel.LEDChannelB);
                canifier.setLEDOutput(35, CANifier.LEDChannel.LEDChannelC);
            } else if (BallSensor.getOutput() == 2) {
                canifier.setLEDOutput(185, CANifier.LEDChannel.LEDChannelA);
                canifier.setLEDOutput(242, CANifier.LEDChannel.LEDChannelB);
                canifier.setLEDOutput(61, CANifier.LEDChannel.LEDChannelC);
            } else if (BallSensor.getOutput() == 3) {
                canifier.setLEDOutput(139, CANifier.LEDChannel.LEDChannelA);
                canifier.setLEDOutput(242, CANifier.LEDChannel.LEDChannelB);
                canifier.setLEDOutput(61, CANifier.LEDChannel.LEDChannelC);

            } else if (BallSensor.getOutput() == 4) {
                canifier.setLEDOutput(91, CANifier.LEDChannel.LEDChannelA);
                canifier.setLEDOutput(242, CANifier.LEDChannel.LEDChannelB);
                canifier.setLEDOutput(61, CANifier.LEDChannel.LEDChannelC);
            } else if (BallSensor.getOutput() == 5) {
                canifier.setLEDOutput(48, CANifier.LEDChannel.LEDChannelA);
                canifier.setLEDOutput(201, CANifier.LEDChannel.LEDChannelB);
                canifier.setLEDOutput(56, CANifier.LEDChannel.LEDChannelC);
            }
        }

    }

}

