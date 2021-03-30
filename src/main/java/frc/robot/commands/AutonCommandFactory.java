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

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
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

    public Command getIntakeIndexer(Intake intake, Indexer indexer) {
        return new ParallelCommandGroup(new RunIntake(intake, 0.5), new Index(indexer, 0.2));
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
        ParallelCommandGroup runShooter = new ParallelCommandGroup(new OpenStopper(stopper),
                new Index(indexer, 0.5, true));
        ParallelCommandGroup shoot = new ParallelCommandGroup(new RunTurretVision(turret, 0.8),
                new Shoot(0.65, shooter), new WaitCommand(2).andThen(runShooter));
        return new ParallelRaceGroup(new WaitCommand(seconds), shoot); // .andThen(new CloseStopper(stopper, indexer));
    }

    public Command launch(double seconds, Stopper stopper, Indexer indexer) {
        return new SequentialCommandGroup(new OpenStopper(stopper),
                new ParallelRaceGroup(new WaitCommand(seconds), new Index(indexer, 0.5, true)),
                new CloseStopper(stopper, indexer));
    }

    /**
     * Run the intake and indexer subsystem.
     * 
     * @param intakeSpeed percentage speed 0-1
     * @param indexSpeed  percentage speed 0-1
     * @param intake
     * @param indexer
     * @return
     */
    public Command runIntakeIndex(double intakeSpeed, double indexSpeed, Intake intake, Indexer indexer) {
        return new ParallelCommandGroup(new RunIntake(intake, intakeSpeed), new Index(indexer, indexSpeed));
    }

    /**
     * Resets the encoders and the odometry of the drivetrain to (0, 0, angle)
     * 
     * @param angle      new initial odometry angle
     * @param drivetrain
     * @return
     */
    public Command resetOdometry(double angle, Drivetrain drivetrain) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(angle)))),
            new InstantCommand(drivetrain::resetEncoders)
        );
    }

    /**
     * Will constantly run the shooter at a set speed, set the turret at a set
     * elevation, and aim the turret using computer vision.
     * 
     * @param speed     percentage speed 0-1
     * @param elevation percentage elevation
     * @param turret
     * @param shooter
     * @return
     */
    public Command prepShooter(double speed, double elevation, Turret turret, Shooter shooter) {
        return new ParallelCommandGroup(new RunTurretVision(turret, elevation), new Shoot(speed, shooter));
    }

    /**
     * Generate a trajectory to drive the robot in a straight line.
     * 
     * @param distance distance, in meters?, to travel
     * @param maxVelocity
     * @param maxAcceleration
     * @param drivetrain
     * @return
     */
    public Command driveStraight(double distance, double maxVelocity, double maxAcceleration, Drivetrain drivetrain) {
        boolean reversed = (distance > 0);
        distance = Math.abs(distance);
        double angle = (reversed) ? Math.PI : 0;

        System.out.println("\n\n\n" + distance + " - " + angle + "\n\n\n");

        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics, 10);

        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration)
                .setKinematics(DriveConstants.kDriveKinematics).addConstraint(autoVoltageConstraint);
        config.setReversed(reversed);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(angle)),
                List.of(), new Pose2d(distance, 0, new Rotation2d(angle)), config);

        RamseteCommand ramseteCommand = new RamseteCommand(trajectory, drivetrain::getPose,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics, drivetrain::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
                drivetrain::tankDriveVolts, drivetrain);
        
        // TODO: maybe the reason this wasn't working was becasue it wasn't in a command.
        // Perhaps the following would work:
        // return new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose())).andthen(ramseteCommand)

        Command resetEncoders = new InstantCommand(drivetrain::resetEncoders);
        Command setOdometry = new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose()));
        return resetEncoders.andThen(setOdometry).andThen(ramseteCommand);
    }

    /**
     * Generate a complete command cycle for the shooting challenges. Requires interuption to end
     * 
     * @param distance distance in feet to travel
     * @param initial whether this is the first path in the cycle; if true will not drive forward
     * @param drivetrain
     * @param shooter
     * @param turret
     * @param intake
     * @param indexer
     * @param stopper
     * @return
     */
    public Command generateShootSequence(double distance, double reverseDistance, boolean initial, Drivetrain drivetrain, Shooter shooter, Turret turret, Intake intake, Indexer indexer, Stopper stopper) {
        return new SequentialCommandGroup(
            prepShooter(0.65, 1, turret, shooter).raceWith(
                new ConditionalCommand(
                    new WaitCommand(1.5),  // if initial, give shooter time to speed up
                    driveStraight(distance, 2, 0.9, drivetrain),  // if not initial, drive into position
                    () -> initial
                ).andThen(launch(3, stopper, indexer))
            ),
            driveStraight(-reverseDistance, 2, 0.9, drivetrain),
            runIntakeIndex(0.5, 0.1, intake, indexer)
        );
    }

    public Command generateShootSequence(double turretVertical, double turretSpeed, double distance, double reverseDistance, boolean initial, Drivetrain drivetrain, Shooter shooter, Turret turret, Intake intake, Indexer indexer, Stopper stopper) {
        return new SequentialCommandGroup(
            prepShooter(turretSpeed, turretVertical, turret, shooter).raceWith(
                new ConditionalCommand(
                    new WaitCommand(3.5),  // if initial, give shooter time to speed up
                    driveStraight(distance, 2, 0.9, drivetrain),  // if not initial, drive into position
                    () -> initial
                ).andThen(launch(3, stopper, indexer))
            ),
            driveStraight(-reverseDistance, 2, 0.9, drivetrain),
            runIntakeIndex(0.5, 0.1, intake, indexer)
        );
    }

    public Command generateShootSequence(double distance, boolean initial, Drivetrain drivetrain, Shooter shooter, Turret turret, Intake intake, Indexer indexer, Stopper stopper) {
        return generateShootSequence(distance, distance, initial, drivetrain, shooter, turret, intake, indexer, stopper);
    }
}
