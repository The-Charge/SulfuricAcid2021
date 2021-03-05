package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.Cool;
import edu.wpi.first.wpilibj.Solenoid;

public class Cooling implements Subsystem {
    /*
     * The cooling subsystem cools down Falcons.
     */
    private Solenoid coolingSolenoid;
    private Drivetrain m_drivetrain;
    private Shooter m_shooter;

    public Cooling(Drivetrain drivetrain, Shooter shooter) {
        coolingSolenoid = new Solenoid(0, 0);
        m_drivetrain = drivetrain;
        m_shooter = shooter;
        setDefaultCommand(new Cool(this));
        coolingSolenoid.set(false);
    }
    // Put code here to be run every loop

    public void cool() {
        SmartDashboard.putBoolean("Shooter Temp", m_shooter.checkTemp());
        if (m_drivetrain.checkTemp() || m_shooter.checkTemp())
            coolingSolenoid.set(true);
        else
            coolingSolenoid.set(false);
        // Put methods for controlling this subsystem
    }
    // here. Call these from Commands.
}