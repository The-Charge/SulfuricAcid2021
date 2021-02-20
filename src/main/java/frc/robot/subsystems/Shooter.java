package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Shooter implements Subsystem {

public WPI_TalonFX motorShooter;

//set to use TalonSRX for testing, will later change to TalonFX 
                                 //in robotbuilder   


//TODO: Tune PID values, current values based on Plybot
  
    private final static double SPEED_P_CONSTANT = 0.1;
	private final static double SPEED_I_CONSTANT = 0.0001;
	private final static double SPEED_D_CONSTANT = 0.0;
	private final static double SPEED_F_CONSTANT = 0.0;
    
    public double speedP = SPEED_P_CONSTANT;
	public double speedI = SPEED_I_CONSTANT;
	public double speedD = SPEED_D_CONSTANT;
    public double speedF = SPEED_F_CONSTANT;
    

    //public double speedP = SmartDashboard.getNumber("Shooter P", 0.1);
	//public double speedI = SmartDashboard.getNumber("Shooter I", 1);
	//public double speedD = SmartDashboard.getNumber("Shooter D", 1);
    //public double speedF = SPEED_F_CONSTANT;

    
    public final static int PID_SLOT_SPEED_MODE = 0;
    
    public double SHOOTER_INWARD_MULTIPLIER = 0;
    public double SHOOTER_OUTWARD_MULTIPLIER = 0;

    private final int TIMEOUT_MS = 10;
    private static final int MAX_TICKS_PER_SEC = 20000;
    public final double XBOX_INCREASE_DECREASE_SHOOTER_SPEED = 0.02;
	 private double AtSpeed = .1; //this is what the current speed is compared to in Speed() method

    public Shooter() {

    motorShooter = new WPI_TalonFX(0);
    //setDefaultCommand(new Shoot(this, 0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter Speed", isAtSpeed(getCurrentSpeed()));
    }

    public void stop(){
        motorShooter.set(0); 
        motorShooter.setNeutralMode(NeutralMode.Coast);
    }

    public void run(double pow) {    	
        motorShooter.set(pow);     
    }

    public void initSpeedMode() {    	
    	motorShooter.set(ControlMode.Velocity, 0);
        
        motorShooter.config_kP(PID_SLOT_SPEED_MODE, speedP, TIMEOUT_MS);
    	motorShooter.config_kI(PID_SLOT_SPEED_MODE, speedI, TIMEOUT_MS);
    	motorShooter.config_kD(PID_SLOT_SPEED_MODE, speedD, TIMEOUT_MS);
    	motorShooter.config_kF(PID_SLOT_SPEED_MODE, speedF, TIMEOUT_MS);

        motorShooter.selectProfileSlot(PID_SLOT_SPEED_MODE, 0);
        
        motorShooter.set(ControlMode.Velocity, 0);
    }

    public void setPercentSpeedPID(double setSpeed) {
        SmartDashboard.putNumber("Shooter PID Val", setSpeed);
        motorShooter.set(ControlMode.Velocity, MAX_TICKS_PER_SEC * setSpeed);
        //motorShooter.set(ControlMode.PercentOutput, setSpeed);
  
    }
    
    //CHECK: was originally int
    public double getTicksPerSecondLeft(){  
        return motorShooter.getSelectedSensorVelocity();
    }

    //CHECK: was originally int
    public double getTicksPerSecondRight(){
        return motorShooter.getSelectedSensorVelocity();
    }

    public double getCurrentSpeed(){
        return motorShooter.getSelectedSensorVelocity();
    }

    public boolean isAtSpeed(double speed)
    {
        double PercentSpeed = (Math.abs(getCurrentSpeed()/MAX_TICKS_PER_SEC - speed));
        return (PercentSpeed < AtSpeed);
    }
    public boolean checkTemp()
    {
        SmartDashboard.putNumber("Temp", motorShooter.getTemperature());
        if (motorShooter.getTemperature() > 35)
            return true;
        else
            return false;
    }

   
}