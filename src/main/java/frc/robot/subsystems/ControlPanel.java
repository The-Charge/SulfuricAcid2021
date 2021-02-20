/*package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.util.Color;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ControlPanel implements Subsystem {

private WPI_TalonSRX controlPanelMotor;

  private final int MAX_COLORS_TURNED = 26;
  private final double CONTROL_PANEL_MOTOR_SPEED = .5;
  private static final double CONFIDENCE_THRESHOLD = .8;

   private String currentColor = "Unknown";
  private String temp = "Unknown";
  private int ctr = 0;
  private double confidences;
 
  
    public ControlPanel() {
    controlPanelMotor = new WPI_TalonSRX(11);
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
  }
  public void run(double pow) {    	
    controlPanelMotor.set(pow);
}
public void stop(){
  controlPanelMotor.set(0);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
}

public void rotateX(ColorSensor m_colorSensor) {
  SmartDashboard.putNumber("ControlPanel Colors Passed Counter", ctr);
  currentColor = m_colorSensor.getColorString();
  confidences = m_colorSensor.getConfidence();
 
    if (ctr<MAX_COLORS_TURNED){
      controlPanelMotor.set(CONTROL_PANEL_MOTOR_SPEED);
    }

    //everytime the color changes counter goes up

    if (currentColor!= temp && confidences > CONFIDENCE_THRESHOLD) {
      ctr++;
      temp = currentColor;
      
      
    }
}
public void rotateColor(Color desiredColor, ColorSensor colorSensor){
  Color currentColor = colorSensor.getColor();
  confidences = colorSensor.getConfidence();

  if(!desiredColor.equals(currentColor) && confidences > CONFIDENCE_THRESHOLD){ 
    controlPanelMotor.set(CONTROL_PANEL_MOTOR_SPEED);
  }
  
}
public int getCounter(){
  return ctr;
}
public void setCounter(int counter){
  ctr = counter;
}
public void setBrakeMode(){
  controlPanelMotor.setNeutralMode(NeutralMode.Brake);
  }
  public void setCoastMode(){
  controlPanelMotor.setNeutralMode(NeutralMode.Coast);
  }
}
*/