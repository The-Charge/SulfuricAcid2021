package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.ControlPanel;
//import com.revrobotics.ColorMatch;
public class RotationControl extends CommandBase {
    private final ControlPanel m_controlPanel;
    private final ColorSensor m_colorSensor;
    private final int ColorsTurnedMax = 26; 

    public RotationControl(ControlPanel controlPanel,ColorSensor colorSensor) {
        m_controlPanel = controlPanel;
        m_colorSensor = colorSensor;
   
        addRequirements(controlPanel);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_controlPanel.setCounter(0);

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_controlPanel.rotateX(m_colorSensor);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // Since the same number is used in the subsystem, would use that one.
        return (m_controlPanel.getCounter()>ColorsTurnedMax);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_controlPanel.stop();
    }
}