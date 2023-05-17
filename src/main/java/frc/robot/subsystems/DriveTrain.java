// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;
import frc.robot.utils.Debug;

public class DriveTrain extends SubsystemBase {
  
  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);
  
  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);
  
  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  
  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();
  
  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();
  
  private PIDController balancePID;
  private PIDController turnPID;
  
  // Odometry class for tracking robot pose
  private  DifferentialDriveOdometry m_odometry;
  
  // Also show a field diagram
  private  Field2d m_field2d = new Field2d();
  
  
  private NetworkTable nt = NetworkTableInstance.getDefault().getTable(Debug.DT_DEBUG_TABLE);
  private NetworkTableEntry nt_lv = nt.getEntry(Debug.DT_LEFT_VOLTS);
  private NetworkTableEntry nt_rv = nt.getEntry(Debug.DT_RIGHT_VOLTS);
  private NetworkTableEntry nt_x = nt.getEntry(Debug.DT_Y);
  private NetworkTableEntry nt_y = nt.getEntry(Debug.DT_X);
  private NetworkTableEntry nt_rot = nt.getEntry(Debug.DT_ROT);
  private NetworkTableEntry nt_ldist = nt.getEntry(Debug.DT_L_DIST);
  private NetworkTableEntry nt_rdist = nt.getEntry(Debug.DT_R_DIST);
  private NetworkTableEntry nt_wheel_speeds = nt.getEntry(Debug.DT_WHEEL_SPEEDS);
  

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    // We need to invert one side of the DriveTrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);
    
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterInch) / Constants.kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterInch) / Constants.kCountsPerRevolution);
    resetEncoders();
    
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    
    balancePID = new PIDController(Constants.kPBalance, 0, 0);
    
    turnPID = new PIDController(Constants.kPTurn, 0, 0);
  }
  
  
  public PIDController getBalancePIDController() {
    return balancePID;
  }
  public PIDController getTurnPIDController() {
    return turnPID;
  }
  
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }
  
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
  
  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }
  
  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }
  
  public double getLeftDistanceMeter() {
    return m_leftEncoder.getDistance();
  }
  
  public double getRightDistanceMeter() {
    return m_rightEncoder.getDistance();
  }
  
  public double getAverageDistanceMeter() {
    return (getLeftDistanceMeter() + getRightDistanceMeter()) / 2.0;
  }
  
  /**
  * The acceleration in the X-axis.
  *
  * @return The acceleration of the Romi along the X-axis in Gs
  */
  public double getAccelX() {
    return m_accelerometer.getX();
  }
  
  /**
  * The acceleration in the Y-axis.
  *
  * @return The acceleration of the Romi along the Y-axis in Gs
  */
  public double getAccelY() {
    return m_accelerometer.getY();
  }
  
  /**
  * The acceleration in the Z-axis.
  *
  * @return The acceleration of the Romi along the Z-axis in Gs
  */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }
  
  /**
  * Current angle of the Romi around the X-axis.
  *
  * @return The current angle of the Romi in degrees
  */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }
  
  /**
  * Current angle of the Romi around the Y-axis.
  *
  * @return The current angle of the Romi in degrees
  */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }
  
  /**
  * Current angle of the Romi around the Z-axis.
  *
  * @return The current angle of the Romi in degrees
  */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }
  
  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }
  
  /**
  * Returns the currently estimated pose of the robot.
  * @return The pose
  */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  /**
  * Returns the current wheel speeds of the robot.
  * @return The current wheel speeds
  */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), getPose());
  }
  
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(-rightVolts); // We invert this to maintain +ve = forward
    
    nt_lv.setNumber(Debug.round(leftVolts));
    nt_rv.setNumber(Debug.round(rightVolts));
    //SmartDashboard.putNumber("leftVolts", cut(leftVolts));
    //SmartDashboard.putNumber("rightVolts", cut(rightVolts));
    
    m_diffDrive.feed();
  }
  
  
  public void putTrajectoryOnField(Trajectory trajectory) {
    //m_field2d.getObject("traj").setTrajectory(trajectory);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    
    // Also update the Field2D object (so that we can visualize this in sim)
    m_field2d.setRobotPose(getPose());

    var translation = m_odometry.getPoseMeters().getTranslation();
    nt_x.setNumber(Debug.round(translation.getX()));
    nt_y.setNumber(Debug.round(translation.getY()));
    nt_rot.setNumber(Debug.round(translation.getAngle().getDegrees()));

    DifferentialDriveWheelSpeeds ws = getWheelSpeeds();
    double speeds[] = {Debug.round(ws.leftMetersPerSecond), Debug.round(ws.rightMetersPerSecond)};
    nt_wheel_speeds.setDoubleArray(speeds);

    nt_ldist.setNumber(Debug.round(getLeftDistanceMeter()));
    nt_rdist.setNumber(Debug.round(getRightDistanceMeter()));
    
    
  }
}
