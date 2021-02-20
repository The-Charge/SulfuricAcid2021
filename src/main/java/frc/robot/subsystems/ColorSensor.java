package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensor implements Subsystem {
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
   
    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
    */

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();
    private String colorString = "Hello";

    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    
    public ColorSensor() {
      setDefaultCommand(new SenseColor(this));
      m_colorMatcher.addColorMatch(kBlueTarget);
      m_colorMatcher.addColorMatch(kGreenTarget);
      m_colorMatcher.addColorMatch(kRedTarget);
      m_colorMatcher.addColorMatch(kYellowTarget); 
    }

    @Override
    public void periodic() {
      SmartDashboard.putString("ColorSensor Detected Color:", colorString);
    }
    
    public void ColorSensed(){
      Color detectedColor = m_colorSensor.getColor();
      /**
       * Run the color match algorithm on our detected color
       */

       
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

      if (match.color == kBlueTarget) {
        colorString = "Blue";
      } else if (match.color == kRedTarget) {
        colorString = "Red";
      } else if (match.color == kGreenTarget) {
        colorString = "Green";
      } else if (match.color == kYellowTarget) {
        colorString = "Yellow";
      } else {
        colorString = "Change"; 
      }
    }

    public Color getColor(){ //reads the color from the color sensor
      return m_colorSensor.getColor();
    }
    public String getColorString(){ //returns a string of a color
      return colorString;
    }
    public double getConfidence(){  
    //getConfidence method returns the confidence it has of it determining the color it thinks it detects
    Color detectedColors = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColors);
    return match.confidence;
    }

}