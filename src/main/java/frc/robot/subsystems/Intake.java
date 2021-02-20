package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Intake implements Subsystem {

private WPI_TalonSRX intakeMotor;

     private final static double SPEED_P_CONSTANT = 0.05;
	private final static double SPEED_I_CONSTANT = 0.0003;
	private final static double SPEED_D_CONSTANT = 0.0;
	private final static double SPEED_F_CONSTANT = 0.0;
    
    public double speedP = SPEED_P_CONSTANT;
	public double speedI = SPEED_I_CONSTANT;
	public double speedD = SPEED_D_CONSTANT;
    public double speedF = SPEED_F_CONSTANT;
    
    public final static int PID_SLOT_SPEED_MODE = 0;
    
    public double SHOOTER_INWARD_MULTIPLIER = 1;
    public double SHOOTER_OUTWARD_MULTIPLIER = 1;

    private final int TIMEOUT_MS = 10;
    private static final int MAX_TICKS_PER_SEC = 31000;

    public Intake() {

        intakeMotor = new WPI_TalonSRX(9);
        initializeMotor();
    }

    @Override
    public void periodic() {
    
        SmartDashboard.putBoolean("Intake OnBall", onBall());
        SmartDashboard.putNumber("Intake CurrentSpeed", intakeMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("CurrentIntake", intakeMotor.getStatorCurrent());
      
    }

    public void run (double setPower) {
        intakeMotor.set(setPower);
	}

    public void initSpeedMode() {    
        intakeMotor.setInverted(true);	
    	intakeMotor.set(ControlMode.Velocity, 0);
        
        intakeMotor.config_kP(PID_SLOT_SPEED_MODE, speedP, TIMEOUT_MS);
    	intakeMotor.config_kI(PID_SLOT_SPEED_MODE, speedI, TIMEOUT_MS);
    	intakeMotor.config_kD(PID_SLOT_SPEED_MODE, speedD, TIMEOUT_MS);
    	intakeMotor.config_kF(PID_SLOT_SPEED_MODE, speedF, TIMEOUT_MS);

    	intakeMotor.selectProfileSlot(PID_SLOT_SPEED_MODE, 0);
    }

	public void stop() {
        intakeMotor.set(0);
	}

    public void setPercentSpeedPID(double setSpeed) {
    
        SmartDashboard.putNumber("Intake SetSpeed", setSpeed);
        intakeMotor.set(ControlMode.Velocity, MAX_TICKS_PER_SEC * setSpeed);
    
    }

	public void setPercentVBus() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
	}

    //CHECK: was originally int
    public double getTicksPerSecond(){
        return intakeMotor.getSelectedSensorVelocity();
    }
    public void setBrakeMode(){
		intakeMotor.setNeutralMode(NeutralMode.Brake);
    }
	public void setCoastMode(){
        intakeMotor.setNeutralMode(NeutralMode.Coast);
    }
   
	public void initializeMotor()
    {
        intakeMotor.setInverted(true);
        intakeMotor.setSensorPhase(true);

    }
    public boolean onBall()
    { 
        //return (intakeMotor.getStatorCurrent() > 15);
        return true;
    }
    public double intakeCurret()
    {
        return intakeMotor.getStatorCurrent();
    }
  
}
