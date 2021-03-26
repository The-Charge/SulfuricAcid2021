package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class Index extends CommandBase {

    private double m_speed;
    private final Indexer m_indexer;
    private boolean shooterOpen = false;

    public Index(Indexer indexer, double speed) {

        m_indexer = indexer;
        m_speed = speed;
        addRequirements(m_indexer);
    }

    public Index(Indexer indexer, double speed, boolean open) {

        m_indexer = indexer;
        m_speed = speed;
        shooterOpen = open;
        addRequirements(m_indexer);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_indexer.initalizeMotors();
        m_indexer.initSpeedMode();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_indexer.setPercentSpeedPIDSimple(m_speed, shooterOpen);
        // m_indexer.setPercentSpeedPID(m_speed, shooterOpen);
        //SmartDashboard.putNumber("Y AXIS VAL", yAxisVal);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Indexer", false);
        m_indexer.stop();
       // Robot.indexer.setPercentVBus();
    }
}