package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Stopper;

public class OpenStopper extends InstantCommand {
    private final Stopper m_stopper;
    
    public OpenStopper(Stopper stopper) {

    m_stopper = stopper;
        addRequirements(m_stopper);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_stopper.openStopper();
    }
}