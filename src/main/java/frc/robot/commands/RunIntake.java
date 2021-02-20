package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {

    private double m_speed;
    public final Intake m_intake;

    public RunIntake(Intake intake, double speed) {
        m_speed = speed;
        m_intake = intake;
        addRequirements(m_intake);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {     
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_intake.setPercentSpeedPID(m_speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }
}