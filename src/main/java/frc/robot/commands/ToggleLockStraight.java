package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class ToggleLockStraight extends CommandBase {

    private final Drivetrain m_subsystem;

    public ToggleLockStraight(Drivetrain subsystem) {

        m_subsystem = subsystem;
        addRequirements(subsystem);

    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_subsystem.setPercentVBus();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double leftSpeed, rightSpeed, avgSpeed;
        rightSpeed = -RobotContainer.rightJoystick.getY();
        leftSpeed = -RobotContainer.leftJoystick.getY();
        avgSpeed = (rightSpeed + leftSpeed) / 2;

        SmartDashboard.putNumber("DriveTrain RightSpeed: ", rightSpeed);
        SmartDashboard.putNumber("DriveTrain LeftSpeed: ", leftSpeed);
        SmartDashboard.putNumber("AvgSpeed: ", avgSpeed);
        m_subsystem.run(avgSpeed, avgSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_subsystem.stop();
    }
}