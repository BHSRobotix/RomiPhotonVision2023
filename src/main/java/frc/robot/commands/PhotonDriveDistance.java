// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PhotonDriveDistance extends CommandBase {
  
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(3);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(1);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
  
  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(1);
  
  PhotonCamera m_camera;
  DriveTrain m_drive;
  PIDController m_turnPID;
  PIDController m_rangePID;
  Joystick m_stick;
  
  /** Creates a new PhotonTest. */
  public PhotonDriveDistance(DriveTrain drive, Joystick stick) {
    // Use addRequirements() here to declare subsystem dependencies.
    //camera = new PhotonCamera("photonvision");
    m_camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    m_drive = drive;
    m_turnPID = drive.getTurnPIDController();
    m_rangePID = new PIDController(.02, 0,0);
    m_stick = stick;
    
    addRequirements(drive);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PhotonDriveDistane initialize");
    SmartDashboard.putData(m_turnPID);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double speed = -m_stick.getY();
    double rotationSpeed = -m_stick.getZ();
    
    var result = m_camera.getLatestResult();
    SmartDashboard.putBoolean("PhotonTest_targets", result.hasTargets());
    if (result.hasTargets()) {
      double range = PhotonUtils.calculateDistanceToTargetMeters(
      CAMERA_HEIGHT_METERS,
      TARGET_HEIGHT_METERS,
      CAMERA_PITCH_RADIANS,
      Units.degreesToRadians(result.getBestTarget().getPitch()));
      
      speed = m_rangePID.calculate(range, GOAL_RANGE_METERS);
      speed = MathUtil.clamp(speed, -.5, .5);
      SmartDashboard.putNumber("PhotonTest_range", range);
      SmartDashboard.putNumber("PhotonTest_range_errs", m_rangePID.getPositionError());
      
      rotationSpeed = m_turnPID.calculate(result.getBestTarget().getYaw(), 0);
      rotationSpeed = MathUtil.clamp(rotationSpeed, -.5, .5);
      SmartDashboard.putNumber("PhotonTest_turn", m_turnPID.getPositionError());
    } 
    m_drive.arcadeDrive(speed, rotationSpeed);
    
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("PhotonDriveDistance done");
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
