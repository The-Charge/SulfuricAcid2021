package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class ResetTurretEncoder extends CommandBase {
    
    private final Turret m_turret;

    public ResetTurretEncoder(Turret turret) {
        m_turret = turret;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_turret);
      }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_turret.setRawHorizontalPercent(m_turret.DEFAULT_HORIZONTAL_ENCODER_PERCENT);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        if (m_turret.atLimitSwitch()){
                m_turret.checkHorizontalLimitSwitches();
                return true;
        }
        else return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_turret.stopHorizontal();
    }
}