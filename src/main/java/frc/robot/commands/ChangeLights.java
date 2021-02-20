package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

public class ChangeLights extends CommandBase {
    private Lights m_lights;
   
    public ChangeLights(Lights lights) {
        m_lights = lights;
    
        addRequirements(m_lights);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_lights.onStart();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        //m_lights.changeLight();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_lights.onStop();
    }
}