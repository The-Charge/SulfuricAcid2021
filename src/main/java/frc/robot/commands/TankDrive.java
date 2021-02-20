package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.RobotContainer;

public class TankDrive extends CommandBase {

    public final Drivetrain m_subsystem;

    public TankDrive(Drivetrain subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_subsystem.limitCurrent();
        m_subsystem.setPercentVBus();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double leftSpeed, rightSpeed;
        rightSpeed = -RobotContainer.rightJoystick.getY();
        leftSpeed = -RobotContainer.leftJoystick.getY();

        //FIXME: Find new MathUtil import
        //rightSpeed = MathUtil.adjSpeed(rightSpeed);
        //leftSpeed = MathUtil.adjSpeed(leftSpeed);

        SmartDashboard.putNumber("DriveTrain leftSpeed", leftSpeed);
        SmartDashboard.putNumber("DriveTrain rightSpeed", rightSpeed);


        m_subsystem.run(leftSpeed, rightSpeed);
        //m_subsystem.tankDriveVolts(leftSpeed, rightSpeed);
  
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