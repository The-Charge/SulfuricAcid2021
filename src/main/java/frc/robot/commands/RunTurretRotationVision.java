// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionLights;
import frc.robot.subsystems.VisionTargetInterface;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.TurretRotation;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class RunTurretRotationVision extends CommandBase {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private final TurretRotation m_turretRotation;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    private final VisionTargetInterface m_visionTargetInterface;
    private final VisionLights m_visionLights;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    public RunTurretRotationVision(TurretRotation subsystem) {

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        m_turretRotation = subsystem;
        addRequirements(m_turretRotation);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        // TODO: set this up to work with multiple subsystems. Will visionLights be
        // required once or constantly?
        m_visionTargetInterface = null;
        m_visionLights = null;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_visionLights.enable();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_visionTargetInterface.success()) {
            boolean locked = m_turretRotation
                    .turnToRelativeAngle(m_visionTargetInterface.get(VisionTargetInterface.Key.HORIZONTAL_ANGLE));
            SmartDashboard.putString("Turret/visionStatus", (locked) ? "locked" : "homing");
        } else {
            m_turretRotation.gotoPosition(0);
            SmartDashboard.putString("Turret/visionStatus", "none");
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_visionLights.disable();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
