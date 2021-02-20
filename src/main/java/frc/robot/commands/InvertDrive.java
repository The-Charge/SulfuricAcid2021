package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class InvertDrive extends InstantCommand {

    private final Drivetrain m_subsystem;
    public InvertDrive(Drivetrain subsystem) {
        m_subsystem = subsystem;
    }

    // Called once when this command runs
    @Override
    public void initialize() {
        m_subsystem.setReversed(!m_subsystem.getReversed());
    }

}