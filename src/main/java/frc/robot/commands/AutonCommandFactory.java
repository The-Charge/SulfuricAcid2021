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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Stopper;
import frc.robot.subsystems.Turret;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class AutonCommandFactory extends CommandBase {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    public AutonCommandFactory() {

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        // m_subsystem = subsystem;
        // addRequirements(m_subsystem);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
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

    public Command getAutonIntake(Intake intake) {
        return new RunIntake(intake, 0.5);
    }

    public Command getShootingProcess(Stopper stopper, Indexer indexer, Shooter shooter, Turret turret) {
        ParallelCommandGroup shoot = new ParallelCommandGroup(new RunTurretVision(turret, 0.8),
                new OpenStopper(stopper), new Index(indexer, 0.5, true));
        // return new Shoot(0.5, shooter);
        return new ParallelCommandGroup(new Shoot(0.65, shooter), new WaitCommand(2).andThen(shoot));
        // return new ParallelCommandGroup(new OpenStopper(stopper), new Index(indexer,
        // 0.5, true));
    }

    public Command shootFor(double seconds, Stopper stopper, Indexer indexer, Shooter shooter, Turret turret) {
        ParallelCommandGroup runShooter = new ParallelCommandGroup(new OpenStopper(stopper), new Index(indexer, 0.5, true));
        ParallelCommandGroup shoot = new ParallelCommandGroup(new RunTurretVision(turret, 0.8), new Shoot(0.65, shooter), new WaitCommand(2).andThen(runShooter));
        return new ParallelRaceGroup(new WaitCommand(seconds), shoot);
    }
}
