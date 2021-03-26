package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import edu.wpi.first.wpilibj.
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Indexer implements Subsystem {

    private WPI_TalonSRX indexerMotorLF;
    private WPI_TalonSRX indexerMotorRF;
    // FIXME: This is unused. Remove.
    private Intake m_intake;
    private DigitalInput ballIn;
    private int timer;
    private MedianFilter ballFilter = new MedianFilter(10);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final static double SPEED_P_CONSTANT = 0.1;
    private final static double SPEED_I_CONSTANT = 0.00001;
    private final static double SPEED_D_CONSTANT = 0.0;
    private final static double SPEED_F_CONSTANT = 0.0;

    public double speedP = SPEED_P_CONSTANT;
    public double speedI = SPEED_I_CONSTANT;
    public double speedD = SPEED_D_CONSTANT;
    public double speedF = SPEED_F_CONSTANT;

    private Stopper m_stopper;

    public final static int PID_SLOT_SPEED_MODE = 0;

    private final int TIMEOUT_MS = 10;
    private final double BALL_SENSED_IN_SPEED = .4;
    private final double BALL_SENSED_OUT_SPEED = .1;
    private static final int MAX_TICKS_PER_SEC = 130000;

    public Indexer(Stopper stopper) {

        indexerMotorLF = new WPI_TalonSRX(8);

        indexerMotorRF = new WPI_TalonSRX(10);

        ballIn = new DigitalInput(2);
        m_stopper = stopper;
    }

    public void periodic() {

    }

    public void stop() {
        indexerMotorLF.set(0);
        indexerMotorRF.set(0);

    }

    public void initSpeedMode() {
        indexerMotorLF.set(ControlMode.Velocity, 0);
        indexerMotorRF.set(ControlMode.Velocity, 0);

        indexerMotorLF.config_kP(PID_SLOT_SPEED_MODE, speedP, TIMEOUT_MS);
        indexerMotorLF.config_kI(PID_SLOT_SPEED_MODE, speedI, TIMEOUT_MS);
        indexerMotorLF.config_kD(PID_SLOT_SPEED_MODE, speedD, TIMEOUT_MS);
        indexerMotorLF.config_kF(PID_SLOT_SPEED_MODE, speedF, TIMEOUT_MS);
        indexerMotorRF.config_kP(PID_SLOT_SPEED_MODE, speedP, TIMEOUT_MS);
        indexerMotorRF.config_kI(PID_SLOT_SPEED_MODE, speedI, TIMEOUT_MS);
        indexerMotorRF.config_kD(PID_SLOT_SPEED_MODE, speedD, TIMEOUT_MS);
        indexerMotorRF.config_kF(PID_SLOT_SPEED_MODE, speedF, TIMEOUT_MS);

        indexerMotorLF.selectProfileSlot(PID_SLOT_SPEED_MODE, 0);
        indexerMotorRF.selectProfileSlot(PID_SLOT_SPEED_MODE, 0);

        // Not used in competition
        setBrakeMode();
    }

    /**
     * A simplified version of the other one, to only run or not run depending on
     * the ball sensor
     * 
     * @param speed
     */
    public void setPercentSpeedPIDSimple(double speed, boolean shooterOpen) {
        if (!ballSensedIn() && !shooterOpen) {
            speed = 0;
            stop();
        } else {
            indexerMotorLF.set(ControlMode.Velocity, MAX_TICKS_PER_SEC * speed);
            indexerMotorRF.set(ControlMode.Velocity, MAX_TICKS_PER_SEC * speed);
        }

        SmartDashboard.putNumber("Indexer/Setpoint", speed);
        SmartDashboard.putBoolean("Indexer/Ball Sensed In", ballSensedIn());
        SmartDashboard.putNumber("Indexer/Actual Speed", getTicksPerSecondLeft());
    }

    public void setPercentSpeedPID(double setSpeed, boolean shooterOpen) {
        boolean wasBall = wasBallSensedIn();
        if (shooterOpen) {  // || setSpeed < 0) {

        }
        // if (m_stopper.ballSensedOut())
        // {
        // //FIXME: Move the magic numbers (0.1, 0.4) to constants. Document them
        // setSpeed = BALL_SENSED_OUT_SPEED*setSpeed;
        // }
        else if (ballSensedIn()) {
            setSpeed = BALL_SENSED_IN_SPEED * setSpeed;
        } else {
            setSpeed = 0.0;
        }

        SmartDashboard.putNumber("Indexer/Setpoint", setSpeed);
        SmartDashboard.putBoolean("Indexer/Ball Sensed In", ballSensedIn());
        SmartDashboard.putNumber("Indexer/Actual Speed", getTicksPerSecondLeft());

        indexerMotorLF.set(ControlMode.Velocity, MAX_TICKS_PER_SEC * setSpeed);
        // indexerMotorRF.set(ControlMode.Velocity, MAX_TICKS_PER_SEC * setSpeed);
    }

    private boolean wasBallSensedIn() {
        if (ballSensedIn()) {
            timer = 0;
        }
        timer++;
        if (timer < 30)
            return true;
        else
            return false;
    }

    // CHECK: was originally int
    public double getTicksPerSecondLeft() {
        return indexerMotorLF.getSelectedSensorVelocity();
    }

    public void setPercentVBus(double setSpeed) {

    }

    public void initalizeMotors() {
        // indexerMotorRF.follow(indexerMotorLF);
    }

    public void setCoastMode() {
        indexerMotorRF.setNeutralMode(NeutralMode.Coast);
        indexerMotorLF.setNeutralMode(NeutralMode.Coast);
    }

    public void setBrakeMode() {
        indexerMotorRF.setNeutralMode(NeutralMode.Brake);
        indexerMotorLF.setNeutralMode(NeutralMode.Brake);
    }

    public boolean ballSensedIn() {
        double data = ballIn.get() ? 1.0 : 0.0;
        return (ballFilter.calculate(data) == 1.0);
    }
}
