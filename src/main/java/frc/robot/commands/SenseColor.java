/*package frc.robot.commands;

import frc.robot.subsystems.ColorSensor;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SenseColor extends CommandBase {
 
  private final ColorSensor m_colorSensor;


  public SenseColor(ColorSensor colorSensor) {
    m_colorSensor = colorSensor;
    addRequirements(m_colorSensor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      //call the method and remove periodic
      m_colorSensor.ColorSensed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
*/