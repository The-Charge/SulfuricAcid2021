package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

public class TurnOffLights extends CommandBase {
    private Lights m_lights;

        m_lights = lights;
        addRequirements(m_lights);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_lights.onStop();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }
}