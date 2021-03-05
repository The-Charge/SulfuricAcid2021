package frc.robot.commands;

import java.security.PublicKey;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shifters;
import frc.robot.subsystems.Turret;
import frc.robot.Robot;

public class TurretToZero extends CommandBase {

    private final Turret m_turret;
    private boolean done;

    public TurretToZero(Turret turret) {
        m_turret = turret;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_turret);
    }

   
    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        done = false;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        done = m_turret.runHorizontalManual(0.0);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return done;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_turret.stopHorizontal();
    }
}