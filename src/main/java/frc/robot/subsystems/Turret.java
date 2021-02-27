package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.RunTurretVision;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Turret implements Subsystem {
    // -482070
    private static final int H_MIN_ENCODER_TICKS = -482070;  // used to stop turret from rotating past ends
    private static final int H_MAX_ENCODER_TICKS = 484191;
    private static final double H_DEGREES_PER_TICK = 0;
    private static final double H_MIN_DEGREES = H_MIN_ENCODER_TICKS * H_DEGREES_PER_TICK;
    private static final double H_MAX_DEGREES = H_MAX_ENCODER_TICKS * H_DEGREES_PER_TICK;

    private static final double H_TOLERANCE = 2;
    private static final double H_MIN_PERCENT = 0.08;
    private static final double H_MAX_PERCENT = 0.25;

    public final double DEFAULT_HORIZONTAL_ENCODER_PERCENT = 0.2;

    //Constants aquired from CAD team used for trig calculations (millimeters):
    public static final double TURRET_SIDE_A = 244.475;
    public static final double TURRET_SIDE_B = 369.4176;
    private static final int TIMEOUT_MS = 10;
    private static final int horizontalSetpoint = 0;
    private static final int H_TOLERANCE_VISION = 1;
    private final WPI_TalonSRX turretMotor;
    private final Servo elevationServo;
    private final Relay visionLights;
   

    public Turret() {    
        turretMotor = new WPI_TalonSRX(11);
        elevationServo = new Servo(0);
        visionLights = new Relay(0);

        turretMotor.set(ControlMode.PercentOutput, 0);
        turretMotor.setSelectedSensorPosition(0);
        turretMotor.setNeutralMode(NeutralMode.Brake);

        //setDefaultCommand(new RunTurretManual(this));

        setDefaultCommand(new RunTurretVision(this, 0.8));
        SmartDashboard.putString("Vision Status", "disabled");

        //setDefaultCommand(new RunTurretVision(this));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/ticks", turretMotor.getSelectedSensorPosition());
        checkHorizontalLimitSwitches();
    }
    
    public void enableVision() {
        visionLights.set(Value.kForward);
    }

    public void disableVision() {
        visionLights.set(Value.kReverse);
    }

    private double calcActuatorDistance(final double angle) {
        // Running law of cosines on the turret
        double d = Math.sqrt(Math.pow(Turret.TURRET_SIDE_A, 2) + Math.pow(Turret.TURRET_SIDE_B, 2) - 2 * Turret.TURRET_SIDE_A * Turret.TURRET_SIDE_B * Math.cos(Math.toRadians(94.4 - angle)));
    
        // This line subtracts the length of the actuator while not extended
        d -= 218;  // 218 is what the actuator blueprints says is the "Closed Length (hole to hole)"
        // This line changes the normalization from 0-140 to 0-1
        d /= 140;  // 140 is what the actuator blueprints says is the max the actuator can extend from the base
        return d;
    }

    public void checkHorizontalLimitSwitches() {
        if (turretMotor.getSensorCollection().isRevLimitSwitchClosed()) {
            turretMotor.setSelectedSensorPosition((int)H_MAX_ENCODER_TICKS, 0, TIMEOUT_MS);
        } else if (turretMotor.getSensorCollection().isFwdLimitSwitchClosed()) {
            turretMotor.setSelectedSensorPosition((int)H_MIN_ENCODER_TICKS, 0, TIMEOUT_MS);
        }
    }

    public boolean atLimitSwitch() {
        return (turretMotor.getSensorCollection().isRevLimitSwitchClosed()
                || turretMotor.getSensorCollection().isFwdLimitSwitchClosed());
    }

    public void gotoHorizontalAngle(double setpoint) {
        SmartDashboard.putNumber("Turret Angle Offset", setpoint);
        SmartDashboard.putBoolean("Turret Valid Turret Rotation",
            getCurrentHorizontalAngle() + setpoint < H_MIN_DEGREES
            || getCurrentHorizontalAngle() + setpoint > H_MAX_DEGREES
        );

        if (Math.abs(setpoint) > H_TOLERANCE) {
            //FIXME: Move the magic numbers to constants. Document them.
            double percent = Math.abs(setpoint) / 40;
            percent = Math.max(H_MIN_PERCENT, Math.min(H_MAX_PERCENT, percent));
            if (setpoint < 0) { percent = -percent; }
            percent = -percent;
            turretMotor.set(ControlMode.PercentOutput, percent);
        } else {
            turretMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void stopHorizontal() {
        turretMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean runHorizontalManual(double target) {
        double ticks = turretMotor.getSelectedSensorPosition();
        if (ticks < 0) {
            ticks /= Math.abs(H_MIN_ENCODER_TICKS);
        } else {
            ticks /= Math.abs(H_MAX_ENCODER_TICKS);
        }
        // SmartDashboard

        double error = target - ticks;
        double speed = 0.2;

        SmartDashboard.putNumber("Turret Normal", ticks);
        SmartDashboard.putNumber("Turret Target", target);
        //FIXME: Move the magic number (0.03) to a constant. Document it.
        if (Math.abs(error) > 0.03) {
            if (error < 0) { speed = -speed; }
            turretMotor.set(ControlMode.PercentOutput, speed);
            return false;
        } else {
            stopHorizontal();
            return true;
        }
    }

    public void setRawHorizontalPercent(double setpoint) {
        turretMotor.set(ControlMode.PercentOutput, setpoint);
    }

    public double getCurrentHorizontalAngle() {
        return turretMotor.getSelectedSensorPosition();
    }

	public void setRawVertical(double verticalAngle) {
        elevationServo.set(verticalAngle);
	}

	public String getVisionString() {
        if (Math.abs(horizontalSetpoint) < H_TOLERANCE_VISION)
        return "locked";
        else return "homing";
	}

	
}