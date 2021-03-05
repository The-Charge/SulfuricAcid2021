package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Cooling;
import frc.robot.subsystems.Shooter;

/**
 *
 */
public class Cool extends CommandBase {
    private final Cooling m_cooling;

    public Cool(Cooling cooling) {

        m_cooling = cooling;
        addRequirements(m_cooling);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_cooling.cool();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {

    }

}