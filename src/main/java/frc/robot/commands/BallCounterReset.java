package frc.robot.commands;

import frc.robot.subsystems.BallSensor;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BallCounterReset extends CommandBase {     //WE NEVER CALL THIS COMMAND

    private final BallSensor m_BallSensor;

    public BallCounterReset(BallSensor ballSensor) {
        m_BallSensor= ballSensor;
        addRequirements(m_BallSensor);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() { 
        m_BallSensor.setBallsgained();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return true;
    }
}