package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;

public class ClimberUnBrake extends InstantCommand {
    
    private Climber m_climber;
    
    public ClimberUnBrake(Climber climber) {
        m_climber = climber;
        addRequirements(m_climber);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_climber.disengageBrake();
    }
}