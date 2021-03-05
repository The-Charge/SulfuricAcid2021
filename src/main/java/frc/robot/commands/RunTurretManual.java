package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class RunTurretManual extends CommandBase {
    private double m_turretHorizontal;
    private final Turret m_turret;
    //private boolean manualActivated = false;
    
    public RunTurretManual(Turret turret) {
        m_turret = turret;
        addRequirements(m_turret);
    }
    
    public RunTurretManual(Turret turret, double turretHorizontal) {
        m_turret = turret;
        m_turretHorizontal = turretHorizontal;
        addRequirements(m_turret);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        //manualActivated = !manualActivated;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
     
         m_turretHorizontal = -RobotContainer.buttonBox.getY();
         m_turret.runHorizontalManual(-m_turretHorizontal); 
        
      
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_turret.stopHorizontal();
    }
}
