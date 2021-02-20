package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

/**
 *
 */
public class ChangeElevation extends CommandBase {
    private double turretHorizontal, m_turretVertical;
    private final Turret m_turret;
    
    public ChangeElevation(Turret turret, double turretVertical) {
        m_turret = turret;
        m_turretVertical = turretVertical;
        addRequirements(m_turret);
    }
    // Called just before this Command runs the first time
    public void execute() {
        m_turret.setRawVertical(m_turretVertical);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return true;
    }
      
}