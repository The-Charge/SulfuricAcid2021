package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class XPercentSpeed extends InstantCommand {

    private final Drivetrain m_subsystem;
    private final double m_speedMult;   //The coefficient that the speed of the Drivetrain will be multiplied by (between 0 and 1). This number doesn't matter if the function is turning off the drive mode

    public XPercentSpeed(Drivetrain subsystem, double speedMult) {
        m_subsystem = subsystem;
        m_speedMult = speedMult;
    }

    // Called once when this command runs
    @Override
    public void initialize() {
        m_subsystem.setMultiplierSpeed(!m_subsystem.getMultiplierSpeed(), m_speedMult);
    }

}