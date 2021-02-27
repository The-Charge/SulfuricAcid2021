package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain;

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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import static edu.wpi.first.wpilibj.XboxController.Button;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * Command-based is a "declarative" paradigm, very little robot logic should
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  
  private static RobotContainer m_robotContainer = new RobotContainer();

  //SUBSYSTEMS
  //public ControlPanel controlPanel = new ControlPanel();
  //public ColorSensor colorSensor = new ColorSensor();
  public Drivetrain drivetrain = new Drivetrain();
  public Turret turret = new Turret();
  public Lights lights = new Lights(turret);
  public Shifters shifters = new Shifters();
  public Climber climber = new Climber();
  public Intake intake = new Intake();
  public Stopper stopper = new Stopper();
  public Indexer indexer = new Indexer(stopper);
  public Shooter shooter = new Shooter();
  public BallSensor ballSensor = new BallSensor(indexer, stopper);
  public Cooling cooling = new Cooling(drivetrain, shooter);

  //JOYSTICKS
  public static Joystick leftJoystick;
  public static Joystick rightJoystick;
  public static Joystick buttonBox;
  public static XboxController Xbox;

  //JOYSTICK BUTTONS
  public JoystickButton shiftHighWPBtn;
  public JoystickButton shiftHighWHBtn;
  public JoystickButton shiftLowBtn;
  public JoystickButton quarterSpeedBtn;
  public JoystickButton halfSpeedBtn;
  public JoystickButton toggleLockStraightBtn;
  public JoystickButton invertDriveBtn;
  public JoystickButton shootBtn;
  public JoystickButton manualElevation;
  public JoystickButton driveXFeetBtn;
  public JoystickButton climbUp;
  public JoystickButton indexBtn;
  public JoystickButton positionControlBtn;
  public JoystickButton rotationControlBtn;
  public JoystickButton runIntakeBtn;
  public JoystickButton runIntakeInverseBtn;
  public JoystickButton sensorColorBtn;
  public JoystickButton zeroBalls;
  public JoystickButton climbDown;
  public JoystickButton visionOverrideBtn;
  public JoystickButton runIntakeIndexerBtn;
  public JoystickButton aButton;
  public JoystickButton bButton;
  public JoystickButton xButton;
  public JoystickButton yButton;
  public JoystickButton openStopperBtn;

  public boolean realButtonBox = true;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
   
    public RobotContainer() {
    SmartDashboard.putData("TurnOffLights", new TurnOffLights(lights));
      if (realButtonBox) configureButtonBindings();
      smartDashboardButtons();   
  }

  private void smartDashboardButtons() {
 
    // SmartDashboard Buttons
    SmartDashboard.putData("Run Vision", new RunTurretVision(turret, 0.4));
    SmartDashboard.putData("Reset Encoder", new ResetTurretEncoder(turret));
    SmartDashboard.putData("Zero Turret", new TurretToZero(turret));
    SmartDashboard.putData("TankDrive", new TankDrive(drivetrain));
    SmartDashboard.putData("ShiftHigh", new ShiftHigh(shifters));
    SmartDashboard.putData("ShiftLow", new ShiftLow(shifters));
    SmartDashboard.putData("Shoot: default", new Shoot(0.4, shooter));
    //SmartDashboard.putData("TurretCommand", new TurretCommand());
    SmartDashboard.putData("RunIntake: default", new RunIntake(intake, 0.4));
    SmartDashboard.putData("DriveXFeetMM: default", new DriveXFeetMM(30, drivetrain));
    //SmartDashboard.putData("TurnNDegreesAbsolute: default", new TurnNDegreesAbsolute(180));
    SmartDashboard.putData("InvertDrive", new InvertDrive(drivetrain));
    SmartDashboard.putData("QuarterSpeed", new QuarterSpeed(drivetrain));
    //SmartDashboard.putData("RotationControl", new RotationControl(controlPanel, colorSensor));
    //SmartDashboard.putData("PositionControl", new PositionsControl(controlPanel, colorSensor));
    SmartDashboard.putData("Index: default", new Index(indexer, 0.1));
    SmartDashboard.putData("HalfSpeed", new HalfSpeed(drivetrain));
    SmartDashboard.putData("ToggleLockStraight", new ToggleLockStraight(drivetrain));
    //SmartDashboard.putData("ManualTurretElevation: default", new ManualTurretElevation(0));
    //SmartDashboard.putData("ManualTurretElevationDegrees: default", new ManualTurretElevationDegrees(0));
    //SmartDashboard.putData("RunTurretVision", new RunTurretVision());
    //SmartDashboard.putData("RunTurretManual", new RunTurretManual());
    SmartDashboard.putData("ClimberRun: up", new ClimberRun(climber, 0.5));
    SmartDashboard.putData("ClimberRun: down", new ClimberRun(climber, -0.5));
    //SmartDashboard.putData("RotationControl", new RotationControl(controlPanel, colorSensor));
    //SmartDashboard.putData("PositionsControl", new PositionsControl(controlPanel, colorSensor));
    SmartDashboard.putNumber("Degrees:", 0);
    SmartDashboard.putNumber("TurnPID P:", 0.05);
    SmartDashboard.putNumber("TurnPID I:", 0.00004);
    SmartDashboard.putNumber("TurnPID D:", 0.0025);
    SmartDashboard.putData("Set Balls 0", new BallCounterReset(ballSensor));

    
    SmartDashboard.putData("GalacticSearchRedA", new GalacticSearchRedA( drivetrain, intake ));
    SmartDashboard.putData("GalacticSearchRedB", new GalacticSearchRedB( drivetrain, intake ));
    SmartDashboard.putData("GalacticSearchBlueB", new GalacticSearchBlueB( drivetrain, intake ));
    SmartDashboard.putData("GalacticSearchBlueA", new GalacticSearchBlueA( drivetrain, intake ));
    SmartDashboard.putData("BarrelRacing", new BarrelRacing( drivetrain ));
    SmartDashboard.putData("Slalom", new Slalom( drivetrain ));
    SmartDashboard.putData("Bounce", new Bounce( drivetrain ));
    SmartDashboard.putData("AutonCommandFactory", new AutonCommandFactory());

    //SmartDashboard.putData("Reinitialize PIDController:", new ReinitializePIDController());
  }
private void configureButtonBindings() {
    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);
    buttonBox = new Joystick(2);
    Xbox = new XboxController(3);

    //reverse intake
    runIntakeInverseBtn = new JoystickButton(buttonBox, 1);
    runIntakeInverseBtn.whileHeld(new RunIntake(intake, -0.2));
    runIntakeInverseBtn.whileHeld(new Index(indexer, -0.7));
      
    //climb up/climb down
    climbDown = new JoystickButton(buttonBox, 3);
    climbDown.whileHeld(new ClimberRun(climber, -0.6));
    climbUp = new JoystickButton(buttonBox, 2);
    climbUp.whileHeld((new SequentialCommandGroup(new ClimberUnBrake(climber), new WaitCommand(1), new ClimberRun(climber, 0.7))));
    climbUp.whenReleased(new ClimberBrake(climber));
      
    //manualElevation = new JoystickButton(buttonBox, 2);
    //manualElevation.whileHeld(new ManualTurretElevation(0));

    //runIntake
    runIntakeBtn = new JoystickButton(buttonBox, 4);
    runIntakeBtn.whileHeld(new RunIntake(intake, 1));

    //Intake and Indexer
    runIntakeIndexerBtn = new JoystickButton(Xbox, 5);
    runIntakeIndexerBtn.whileHeld(new RunIntake(intake, 0.6));
    runIntakeIndexerBtn.whileHeld(new Index(indexer, 1));
    

    shootBtn = new JoystickButton(Xbox, 6);
    //shootBtn.whileHeld(new ParallelCommandGroup(new OpenStopper(stopper))); indexer, slow speed
    shootBtn.whileHeld(new ParallelCommandGroup(new OpenStopper(stopper), new Index(indexer, 0.5, true)));
    shootBtn.whenReleased(new ParallelCommandGroup (new CloseStopper(stopper, indexer), new Shoot(0, shooter)));

    visionOverrideBtn = new JoystickButton(buttonBox, 8);
    visionOverrideBtn.whenPressed(new RunTurretManual(turret));
     
    positionControlBtn = new JoystickButton(buttonBox, 5);
    //positionControlBtn.whileHeld(new PositionsControl(controlPanel, colorSensor));
    rotationControlBtn = new JoystickButton(buttonBox, 6);
    //rotationControlBtn.whileHeld(new RotationControl(controlPanel, colorSensor));

    //senseColorBtn = new JoystickButton(buttonBox, 5);
    //senseColorBtn.whileHeld(new SenseColor(colorSensor));

    //Drive Train buttons
    
    //left joystick
    toggleLockStraightBtn = new JoystickButton(leftJoystick, 4 );
    toggleLockStraightBtn.whileHeld(new ToggleLockStraight(drivetrain));

    //right joystick
    invertDriveBtn = new JoystickButton(rightJoystick, 2);
    invertDriveBtn.whenPressed(new InvertDrive(drivetrain));
    shiftHighWPBtn = new JoystickButton(rightJoystick, 3);
    shiftHighWPBtn.whenPressed(new ShiftHigh(shifters));
    quarterSpeedBtn = new JoystickButton(rightJoystick, 4);
    quarterSpeedBtn.whenPressed(new QuarterSpeed(drivetrain));
    
    shiftLowBtn = new JoystickButton(rightJoystick, 5);
    shiftLowBtn.whenPressed(new ShiftLow(shifters));
    shiftHighWHBtn = new JoystickButton(rightJoystick, 1);
    shiftHighWHBtn.whenPressed(new ShiftHigh(shifters));
    shiftHighWHBtn.whenReleased(new ShiftLow(shifters));

      aButton = new JoystickButton(Xbox, 1);
      aButton.whenPressed(new Shoot(1, shooter)); 
      aButton.whenPressed(new RunTurretVision(turret, 0.8)); 
      bButton = new JoystickButton(Xbox, 2);
      bButton.whenPressed(new Shoot(.8, shooter));
      bButton.whenPressed(new RunTurretVision(turret, 0.8));
      xButton = new JoystickButton(Xbox, 3);
      xButton.whenPressed(new Shoot(.7, shooter));
      xButton.whenPressed(new RunTurretVision(turret, 0.8));
      yButton = new JoystickButton(Xbox, 4);
      yButton.whenPressed(new Shoot(.48, shooter));
      yButton.whenPressed(new RunTurretVision(turret, 0.4));
      openStopperBtn = new JoystickButton(Xbox, 10);
      openStopperBtn.whenPressed(new OpenStopper(stopper));



   
    // m_chooser.addOption("GalacticSearchRedA", new GalacticSearchRedA( drivetrain, intake ));
    // m_chooser.addOption("GalacticSearchRedB", new GalacticSearchRedB( drivetrain, intake ));
    // m_chooser.addOption("GalacticSearchBlueB", new GalacticSearchBlueB( drivetrain, intake ));
    // m_chooser.addOption("GalacticSearchBlueA", new GalacticSearchBlueA( drivetrain, intake ));
    // m_chooser.addOption("BarrelRacing", new BarrelRacing( drivetrain ));
    // m_chooser.addOption("Slalom", new Slalom( drivetrain ));
    // m_chooser.addOption("Bounce", new Bounce( drivetrain ));
    m_chooser.setDefaultOption("Slalom", new Slalom(drivetrain));

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    // return null;
    // return new Slalom(drivetrain);
    return m_chooser.getSelected();
  }
  
  public Intake getIntake(){
    return intake;
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

}
