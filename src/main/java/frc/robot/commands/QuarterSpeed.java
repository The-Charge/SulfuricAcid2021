package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class QuarterSpeed extends TankDrive {

    public QuarterSpeed(Drivetrain subsystem) {
        super(subsystem);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_subsystem.setPercentVBus();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double leftSpeed, rightSpeed;
        rightSpeed = -RobotContainer.rightJoystick.getY() * 0.25;
        leftSpeed = -RobotContainer.leftJoystick.getY() * 0.25;
        m_subsystem.run(leftSpeed, rightSpeed);

        // FIXME: Find MathUtil Import
        // rightSpeed = MathUtil.adjSpeed(rightSpeed);
        // leftSpeed = MathUtil.adjSpeed(leftSpeed);
        /*
         * SmartDashboard.putNumber("Drivetrain QuarterSpeedL", leftSpeed);
         * SmartDashboard.putNumber("Drivetrain QuarterSpeedR", rightSpeed);
         */
        m_subsystem.run(leftSpeed, rightSpeed);
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