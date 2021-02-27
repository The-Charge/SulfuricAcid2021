package frc.robot.subsystems;

//import frc.robot.MathUtil;
import edu.wpi.first.wpilibj.SerialPort.Port; //might change to I2C
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TankDrive;

public class Drivetrain extends SubsystemBase {

  public WPI_TalonFX leftFrontMotor = new WPI_TalonFX(14);
  public WPI_TalonFX leftMidMotor = new WPI_TalonFX(13);
  private WPI_TalonFX leftBackMotor = new WPI_TalonFX(15);

  public WPI_TalonFX rightFrontMotor = new WPI_TalonFX(2);
  private WPI_TalonFX rightMidMotor = new WPI_TalonFX(1);
  private WPI_TalonFX rightBackMotor = new WPI_TalonFX(3);

  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(leftFrontMotor, leftMidMotor,
      leftBackMotor);

  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(rightFrontMotor, rightMidMotor,
      rightBackMotor);

  // The robot's drive
  // private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors,
  // m_rightMotors);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(Port.kUSB);

  // Odometry class for tracking robot pose
  public final DifferentialDriveOdometry m_odometry;
  //private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // PID Constants (all values still need to be changed, these are values for
  // plybot)
  private static final double SPEED_P_CONSTANT = 0.25;
  private static final double SPEED_I_CONSTANT = 0.0001; // lowered
  private static final double SPEED_D_CONSTANT = 0.0;
  private static final double SPEED_F_CONSTANT = 0.12;

  private static final int TIMEOUT_MS = 10;
  private static final int MAX_TICKS_PER_SECOND = 9000; // Plybot = 9000 Sulfuric = 200000
  private static final int TICKS_PER_FOOT = 5270; // Plybot = 5270 Sulfuric = 9560

  // Motion Magic (all values still need to be changed, these are values for
  // plybot)
  public double MotionMagicP = .8;
  public double MotionMagicI = 0.0;
  public double MotionMagicD = 0.001;
  public double MotionMagicF = 0.65;
  public int MotionMagicAcceleration = 2500;
  public int MotionMagicVelocity = 5000;
  public int MotionMagicPIDIndex = 0;
  public int MotionMagicPIDSlot = 0;
  public double MotionMagicDistance;
  public final double MAX_TEMP = 35;
  // public double correctionR = 1.02;

  public int smoothing = 4;

  /*
   * Decent PID Values for Resistance: P: 0.05 I: 0.00004 D: 0.0025
   */
  // These need to be tuned
  private static double PIDTURN_P = 0.05;
  private static double PIDTURN_I = 0.00004;
  private static double PIDTURN_D = 0.0025;

  public PIDController pidController;

  // private static final AHRS ahrs = new AHRS(Port.kMXP);

  private static boolean isReversed = false;

  public Drivetrain() {
    initializeMotors();
     setBrakeMode();
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    pidController = new PIDController(PIDTURN_P, PIDTURN_I, PIDTURN_D);

    setDefaultCommand(new TankDrive(this));
  }

  @Override
  public void periodic() {
    // Put code here to be run every loop
    SmartDashboard.putNumber("Drive Train/left", leftFrontMotor.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Drive Train/right", rightFrontMotor.getSelectedSensorPosition(0));
    m_odometry.update(Rotation2d.fromDegrees(getHeading()),
        leftFrontMotor.getSelectedSensorPosition(0) * DriveConstants.kEncoderDistancePerPulse,
        rightFrontMotor.getSelectedSensorPosition(0) * DriveConstants.kEncoderDistancePerPulse);
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public void run(double l, double r) {
    double leftSpeed = l;
    double rightSpeed = r;

    if (isReversed) {
      leftSpeed = -1 * leftSpeed;
      rightSpeed = -1 * rightSpeed;
    }
    SmartDashboard.putNumber("Drivetrain leftSpeed", leftSpeed);
    SmartDashboard.putNumber("Drivetrain rightSpeed", rightSpeed);
    runRaw(rightSpeed, leftSpeed);
  }

  public void runRaw(double rightSpeed, double leftSpeed) {
    m_rightMotors.set(rightSpeed);
    m_leftMotors.set(leftSpeed);
  }

  public void stop() {
    leftFrontMotor.set(0);
    rightFrontMotor.set(0);
  }

  public void setPercentVBus() {

    leftFrontMotor.set(ControlMode.PercentOutput, 0);
    rightFrontMotor.set(ControlMode.PercentOutput, 0);
  }

  public void initSpeedMode() {
    leftFrontMotor.set(ControlMode.Velocity, 0);
    rightFrontMotor.set(ControlMode.Velocity, 0);

    // Assigned PID constants to the motors.
    leftFrontMotor.config_kP(1, SPEED_P_CONSTANT, TIMEOUT_MS);
    leftFrontMotor.config_kI(1, SPEED_I_CONSTANT, TIMEOUT_MS);
    leftFrontMotor.config_kD(1, SPEED_D_CONSTANT, TIMEOUT_MS);
    leftFrontMotor.config_kF(1, SPEED_F_CONSTANT, TIMEOUT_MS);

    rightFrontMotor.config_kP(1, SPEED_P_CONSTANT, TIMEOUT_MS);
    rightFrontMotor.config_kI(1, SPEED_I_CONSTANT, TIMEOUT_MS);
    rightFrontMotor.config_kD(1, SPEED_D_CONSTANT, TIMEOUT_MS);
    rightFrontMotor.config_kF(1, SPEED_F_CONSTANT, TIMEOUT_MS);

  }

  public void setControlMode(ControlMode mode) {
    leftFrontMotor.set(mode, 0);
    rightFrontMotor.set(mode, 0);
  }

  public ControlMode getControlMode() {
    return leftFrontMotor.getControlMode();
  }

  public void setPercentSpeedPID(double setSpeedL, double setSpeedR) {
    setSpeedR = MathUtil.clamp(setSpeedR, -1, 1);
    setSpeedL = MathUtil.clamp(setSpeedL, -1, 1);
    leftFrontMotor.set(ControlMode.Velocity, MAX_TICKS_PER_SECOND * setSpeedL);
    rightFrontMotor.set(ControlMode.Velocity, MAX_TICKS_PER_SECOND * setSpeedR);
  }

  public boolean getReversed() {
    return isReversed;
  }

  public void setReversed(boolean r) {
    isReversed = r;
  }

  public void writePIDs(double output) {
    leftFrontMotor.pidWrite(output);
    rightFrontMotor.pidWrite(-output);
  }

  public void setCoastMode() {
    leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    rightFrontMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void setBrakeMode() {
    leftFrontMotor.setNeutralMode(NeutralMode.Brake);
    rightFrontMotor.setNeutralMode(NeutralMode.Brake);
  }

  // Motion Magic for DriveXFeet command
  public void MotionMagicInit(double distance) {
    // rightFrontMotor.follow(leftFrontMotor);

    leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, MotionMagicPIDIndex, TIMEOUT_MS);
    rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, MotionMagicPIDIndex, TIMEOUT_MS);

    // Setting min and max outputs (new code)
    leftFrontMotor.configNominalOutputForward(0, TIMEOUT_MS);
    leftFrontMotor.configNominalOutputReverse(0, TIMEOUT_MS);
    leftFrontMotor.configPeakOutputForward(1, TIMEOUT_MS);
    leftFrontMotor.configPeakOutputReverse(-1, TIMEOUT_MS);

    rightFrontMotor.configNominalOutputForward(0, TIMEOUT_MS);
    rightFrontMotor.configNominalOutputReverse(0, TIMEOUT_MS);
    rightFrontMotor.configPeakOutputForward(1, TIMEOUT_MS);
    rightFrontMotor.configPeakOutputReverse(-1, TIMEOUT_MS);

    leftFrontMotor.selectProfileSlot(MotionMagicPIDSlot, MotionMagicPIDIndex);
    rightFrontMotor.selectProfileSlot(MotionMagicPIDSlot, MotionMagicPIDIndex);

    leftFrontMotor.config_kP(0, MotionMagicP, TIMEOUT_MS);
    leftFrontMotor.config_kI(0, MotionMagicI, TIMEOUT_MS);
    leftFrontMotor.config_kD(0, MotionMagicD, TIMEOUT_MS);
    leftFrontMotor.config_kF(0, MotionMagicF, TIMEOUT_MS);

    rightFrontMotor.config_kP(0, MotionMagicP, TIMEOUT_MS);
    rightFrontMotor.config_kI(0, MotionMagicI, TIMEOUT_MS);
    rightFrontMotor.config_kD(0, MotionMagicD, TIMEOUT_MS);
    rightFrontMotor.config_kF(0, MotionMagicF, TIMEOUT_MS);

    leftFrontMotor.configMotionAcceleration(MotionMagicAcceleration, TIMEOUT_MS);
    leftFrontMotor.configMotionCruiseVelocity(MotionMagicVelocity, TIMEOUT_MS);

    // rightFrontMotor.configMotionAcceleration((int)(correctionR*MotionMagicAcceleration),
    // TIMEOUT_MS);
    rightFrontMotor.configMotionAcceleration(MotionMagicAcceleration, TIMEOUT_MS);
    rightFrontMotor.configMotionCruiseVelocity(MotionMagicVelocity, TIMEOUT_MS);

    leftFrontMotor.setSelectedSensorPosition(0, MotionMagicPIDIndex, TIMEOUT_MS);
    rightFrontMotor.setSelectedSensorPosition(0, MotionMagicPIDIndex, TIMEOUT_MS);

    // Smoothing factor
    leftFrontMotor.configMotionSCurveStrength(smoothing);
    rightFrontMotor.configMotionSCurveStrength(smoothing);

    MotionMagicDistance = distance * TICKS_PER_FOOT;
    leftFrontMotor.set(ControlMode.MotionMagic, MotionMagicDistance);
    rightFrontMotor.set(ControlMode.MotionMagic, MotionMagicDistance);

    // rightFrontMotor.set(ControlMode.MotionMagic,
    // correctionR*MotionMagicDistance);
  }

  public boolean isAtPIDDestination() {
    return (Math.abs(this.leftFrontMotor.getSelectedSensorPosition(MotionMagicPIDIndex) - MotionMagicDistance) < 500)
        || (Math.abs(this.rightFrontMotor.getSelectedSensorPosition(MotionMagicPIDIndex) - MotionMagicDistance) < 500);// ||
                                                                                                                       // this.leftFrontMotor.getSelectedSensorPosition(MotionMagicPIDIndex)
                                                                                                                       // <
                                                                                                                       // -MotionMagicDistance
                                                                                                                       // +
                                                                                                                       // 6000)
  }

  public void ResestEncoder() {
    leftFrontMotor.setSelectedSensorPosition(0, 0, TIMEOUT_MS);
    rightFrontMotor.setSelectedSensorPosition(0, 0, TIMEOUT_MS);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftFrontMotor.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse,
        rightFrontMotor.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Controls the left and right sides of the drive directly with `s.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    SmartDashboard.putNumber("Left Encoder", leftFrontMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Encoder", rightFrontMotor.getSelectedSensorPosition());
    //m_diffDrive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    leftFrontMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
  }

  public void setEncoders(int left, int right) {
    leftFrontMotor.setSelectedSensorPosition(left);
    rightFrontMotor.setSelectedSensorPosition(right);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
    //return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void initializeMotors() {
    rightBackMotor.setInverted(true);
    rightMidMotor.setInverted(true);
    rightFrontMotor.setInverted(true);

    rightBackMotor.follow(rightFrontMotor);
    rightMidMotor.follow(rightFrontMotor);
    leftBackMotor.follow(leftFrontMotor);
    leftMidMotor.follow(leftFrontMotor);

    rightBackMotor.configOpenloopRamp(0.5);
    rightFrontMotor.configOpenloopRamp(0.5);
    rightMidMotor.configOpenloopRamp(0.5);
    leftBackMotor.configOpenloopRamp(0.5);
    leftFrontMotor.configOpenloopRamp(0.5);
    leftMidMotor.configOpenloopRamp(0.5);
  }

  public boolean checkTemp() {
    if (rightFrontMotor.getTemperature() > MAX_TEMP || rightMidMotor.getTemperature() > MAX_TEMP
        || rightBackMotor.getTemperature() > MAX_TEMP || leftFrontMotor.getTemperature() > MAX_TEMP
        || leftMidMotor.getTemperature() > MAX_TEMP || leftBackMotor.getTemperature() > MAX_TEMP)
      return true;
    else
      return false;
  }

  public void limitCurrent() {

    rightFrontMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5));
    rightMidMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5));
    rightBackMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5));

    leftFrontMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5));
    leftMidMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5));
    leftBackMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5));
  }

}
