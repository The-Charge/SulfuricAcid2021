package frc.robot.commands;

import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ClimberBrake extends InstantCommand {

    private Climber m_climber;
    public ClimberBrake(Climber climber) {
        m_climber = climber;
        addRequirements(m_climber);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_climber.engageBrakes();
    }
}