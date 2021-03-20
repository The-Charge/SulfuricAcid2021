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

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.Drivetrain;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class Bounce extends SequentialCommandGroup {

    Drivetrain m_drivetrain;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    public Bounce(Drivetrain drivetrain){

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        m_drivetrain = drivetrain;
        m_drivetrain.setBrakeMode();
        addCommands(
            // getSegment(
            //     new Pose2d(0, 0, new Rotation2d(0)),
            //     new Pose2d(2, 0, new Rotation2d(0)),
            //     List.of(new Translation2d(1, 0.5)),
            //     false
            // ),
            // new WaitCommand(1),
            // new InstantCommand(() -> m_drivetrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(Math.PI)))),
            // getSegment(
            //     new Pose2d(0, 0, new Rotation2d(Math.PI)),
            //     new Pose2d(2, 0, new Rotation2d(Math.PI)),
            //     List.of(),
            //     true
            // )
            getSegment(
                new Pose2d(0, 0, new Rotation2d(0)),
                new Pose2d(1.1, 1.2, new Rotation2d(Math.PI/2)),
                List.of(),
                false
            ),
            new InstantCommand(() -> m_drivetrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(Math.PI)))),
            getSegment(
                new Pose2d(0, 0, new Rotation2d(Math.PI)),
                //new Pose2d(3.4, 1.2, new Rotation2d(Math.PI)),
                new Pose2d(0.3, 2.35, new Rotation2d(0)),
                List.of(new Translation2d(1.27, 0.45), new Translation2d(2.5, 1.8)),
                true
            ),
            new InstantCommand(() -> m_drivetrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
            getSegment(
                new Pose2d(0, 0, new Rotation2d(0)),
                //new Pose2d(3.4, 1.2, new Rotation2d(Math.PI)),
                new Pose2d(0, 2.2, new Rotation2d(Math.PI)),
                List.of(new Translation2d(2.2, 0.2), new Translation2d(2.2, 1.75)),
                false
            ),
            new InstantCommand(() -> m_drivetrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(Math.PI)))),
            getSegment(
                new Pose2d(0, 0, new Rotation2d(Math.PI)),
                //new Pose2d(3.4, 1.2, new Rotation2d(Math.PI)),
                new Pose2d(1.1, 0.7, new Rotation2d(4*Math.PI/3)),
                List.of(),
                true
            )
            // new WaitCommand(1),
            // new InstantCommand(() -> m_drivetrain.resetOdometry(new Pose2d(3.4, 1.2, new Rotation2d(Math.PI/2)))),
            // getSegment(
            //     new Pose2d(3.4, 1.2, new Rotation2d(Math.PI/2)),
            //     new Pose2d(5.33, 1.2, new Rotation2d(Math.PI/2)),
            //     List.of(new Translation2d(3.4, -1.5), new Translation2d(5.33, -1.5)),
            //     false
            // ),
            // new WaitCommand(1),
            // new InstantCommand(() -> m_drivetrain.resetOdometry(new Pose2d(5.33, 1.2, new Rotation2d(Math.PI/2)))),
            // getSegment(
            //     new Pose2d(5.33, 1.2, new Rotation2d(Math.PI/2 + Math.PI)),
            //     new Pose2d(6.5, 0, new Rotation2d(-Math.PI/4 + Math.PI)),
            //     List.of(), 
            //     true
            // )
        );
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }

    private Command getSegment(Pose2d origin, Pose2d end, List<Translation2d> cords, boolean reversed) {
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    DriveConstants.ksVolts, 
                    DriveConstants.kvVoltSecondsPerMeter, 
                    DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                10);

        TrajectoryConfig config =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond, 
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint)
                .setReversed(reversed);
        
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            origin, cords, end, config);
        
        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            m_drivetrain::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            m_drivetrain::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            m_drivetrain::tankDriveVolts,
            m_drivetrain);
    
        m_drivetrain.resetOdometry(trajectory.getInitialPose());
        return ramseteCommand;
    }

    private Command getAutonomous(String seg, boolean reversed) {
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(DriveConstants.ksVolts, 
                                           DriveConstants.kvVoltSecondsPerMeter, 
                                           DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                10);
    
        TrajectoryConfig config =
            new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
                                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint);

    
        config.setReversed(reversed);
         
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.PI)),
            List.of(
                //new Translation2d(1.0, 0.5)
                //new Translation2d(1.0, -0.25),
                //new Translation2d(1.5, 0)
            ),
            new Pose2d(2.0, 0, new Rotation2d(Math.PI)),
            config);
          
          
          /*
          //String trajectoryJSON = "paths/test.wpilib.json";
          String trajectoryJSON = "paths/bounce" + seg + ".wpilib.json";
          Trajectory trajectory = new Trajectory();
          try { 
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
          } catch (IOException ex) {
            //DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            System.out.println("Unable to open");
          }
          */
          
          
        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            m_drivetrain::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            m_drivetrain::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            m_drivetrain::tankDriveVolts,
            m_drivetrain);
    
        m_drivetrain.resetOdometry(trajectory.getInitialPose());
    
        return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));
      } 
    
}
