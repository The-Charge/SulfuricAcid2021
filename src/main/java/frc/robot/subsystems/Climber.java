package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Climber extends SubsystemBase {

    
private WPI_TalonSRX climberMotor;
private DoubleSolenoid climberBrakeDoubleSolenoid;
private boolean goingUp;
private final int PEAK_CURRENT_LIMIT = 40;// don't activate current limit until current exceeds 30 A ...
private final int PEAK_CURRENT_DURATION = 100; // ... for at least 100 ms
private final int CONTINUOUS_CURRENT_LIMIT = 35;// once current-limiting is actived, hold at 20A

    final int TIMEOUT_MS = 10;   //needs tuning
    private final static int MAX_TICKS_PER_SEC = 934;
   

    public Climber() {
climberMotor = new WPI_TalonSRX(12);
climberBrakeDoubleSolenoid = new DoubleSolenoid(0, 3, 4);
        
goingUp = false;
//setDefaultCommand(new ClimberBrake(this));

    }

    public void stopMotor(){
        climberMotor.set(ControlMode.PercentOutput,0);
    }

    public void engageBrakes(){
        climberBrakeDoubleSolenoid.set(Value.kForward);
    }
    public void disengageBrake()
    {
        climberBrakeDoubleSolenoid.set(Value.kReverse);
    }

    public void set(double percentSpeed)
    {
        climberBrakeDoubleSolenoid.set(Value.kOff);
        if(percentSpeed > 0)
            goingUp = true;
        else
            goingUp = false;
        climberMotor.set(ControlMode.PercentOutput, percentSpeed);
    }
    public void setPercentVBus()
    {
        climberMotor.set(ControlMode.PercentOutput, 0);
    }
    public void setBrakeMode(){
		climberMotor.setNeutralMode(NeutralMode.Brake);
    }
    public void setCoastMode(){
		climberMotor.setNeutralMode(NeutralMode.Coast);
    }
	public void initalizeMotor(){
	
	climberMotor.setInverted(true);
	}

    public double getCurrent()
    {
        return climberMotor.getSupplyCurrent();
    }
 
    public void limitCurrent()
    {
        //TODO find and test values for constants
        //Don't know the exact max current number
        climberMotor.configPeakCurrentLimit(PEAK_CURRENT_LIMIT); // don't activate current limit until current exceeds 30 A ...
        climberMotor.configPeakCurrentDuration(PEAK_CURRENT_DURATION); // ... for at least 100 ms
        climberMotor.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT); // once current-limiting is actived, hold at 20A
        climberMotor.enableCurrentLimit(true);
        //Enabling Current Limit means the Talon SRX will automatically monitor the current output
        //  and restrict the output with a limit cap.
    }

}
    
