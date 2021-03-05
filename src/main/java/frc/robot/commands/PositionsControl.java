/*
 * package frc.robot.commands;
 * 
 * import com.revrobotics.ColorMatch; import
 * edu.wpi.first.wpilibj.DriverStation; import
 * edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; import
 * edu.wpi.first.wpilibj.util.Color; import frc.robot.subsystems.ColorSensor;
 * import frc.robot.subsystems.ControlPanel; import
 * edu.wpi.first.wpilibj2.command.CommandBase;
 * 
 * public class PositionsControl extends CommandBase { private final
 * ControlPanel m_controlPanel; private final ColorSensor m_colorSensor; //=
 * (0.143, 0.427, 0.429);
 * 
 * private Color targetColor; private String targetString; private Color
 * desiredColor = ColorMatch.makeColor(0.143, 0.427, 0.429);
 * 
 * public PositionsControl(ControlPanel controlPanel, ColorSensor colorSensor) {
 * m_controlPanel = controlPanel; m_colorSensor = colorSensor;
 * 
 * addRequirements(m_controlPanel);
 * 
 * }
 * 
 * // Called just before this Command runs the first time
 * 
 * @Override public void initialize() { String gameData =
 * DriverStation.getInstance().getGameSpecificMessage();
 * 
 * if (gameData.equals("B")) desiredColor = ColorMatch.makeColor(0.143, 0.427,
 * 0.429); if (gameData.equals("G")) desiredColor = ColorMatch.makeColor(0.197,
 * 0.561, 0.240); if (gameData.equals("R")) desiredColor =
 * ColorMatch.makeColor(0.561, 0.232, 0.114); if (gameData.equals("Y"))
 * desiredColor = ColorMatch.makeColor(0.361, 0.524, 0.113);
 * 
 * targetColor = desiredColor; targetString = gameData;
 * 
 * }
 * 
 * // Called repeatedly when this Command is scheduled to run
 * 
 * @Override public void execute() { //
 * Robot.m_controlPanel.rotateColor(targetColor);
 * m_controlPanel.rotateColor(targetColor, m_colorSensor); }
 * 
 * // Make this return true when this Command no longer needs to run execute()
 * 
 * @Override public boolean isFinished() {
 * SmartDashboard.putBoolean("Is at target",
 * targetString.equals(m_colorSensor.getColorString().substring(0, 1))); return
 * targetString.equals(m_colorSensor.getColorString().substring(0, 1)); }
 * 
 * // Called once after isFinished returns true
 * 
 * @Override public void end(boolean interupted ) { m_controlPanel.stop();
 * 
 * } }
 */