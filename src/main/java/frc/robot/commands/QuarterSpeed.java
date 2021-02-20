package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class QuarterSpeed extends InstantCommand {

    private final Drivetrain m_subsystem;

    public QuarterSpeed(Drivetrain subsystem) {
        m_subsystem = subsystem;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_subsystem.setQuarterSpeed(!m_subsystem.getQuarterSpeed());
    }
}