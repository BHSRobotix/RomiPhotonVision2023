// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

public class PhotonFollow extends CommandBase {
  PhotonCamera m_camera;
  DriveTrain m_drive;
  PIDController m_turnPID;
  PIDController m_rangePID;
  CommandXboxController xboxController;

  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(3);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(1);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
  
  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(1);
  
 
  public PhotonFollow(DriveTrain drive, CommandXboxController stick) {
      m_camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
      m_drive = drive;
      m_turnPID = drive.getTurnPIDController();
      m_rangePID = new PIDController(.3, 0,0);
      xboxController = stick;
  
      addRequirements(drive);
    }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PhotonFollow initialize");
    SmartDashboard.putData(m_turnPID);

    SmartDashboard.putNumber("PhotonTest_range_target", GOAL_RANGE_METERS);

  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationSpeed;
    double speed;

    var result = m_camera.getLatestResult();
    SmartDashboard.putBoolean("PhotonTest_targets", result.hasTargets());
    if (result.hasTargets()) {
      double range = PhotonUtils.calculateDistanceToTargetMeters(
      CAMERA_HEIGHT_METERS,
      TARGET_HEIGHT_METERS,
      CAMERA_PITCH_RADIANS,
      Units.degreesToRadians(result.getBestTarget().getPitch()));

      SmartDashboard.putNumber("PhotonTest_range", range);

      SmartDashboard.putNumber("PhotonTest_range_abs", Math.abs(range));
      if (Math.abs(range) > 25) {
      //if (m_rangePID.getPositionError() < Math.abs(25)) {
        speed = m_rangePID.calculate(range, GOAL_RANGE_METERS);
        speed = MathUtil.clamp(speed, -.5, .5);
      } else {
        speed = 0;
      }
      SmartDashboard.putNumber("PhotonTest_range_errs", m_rangePID.getPositionError());
      
      rotationSpeed = m_turnPID.calculate(result.getBestTarget().getYaw(), 0);
      rotationSpeed = MathUtil.clamp(rotationSpeed, -.5, .5);

      SmartDashboard.putNumber("PhotonTest_speed", speed);
      SmartDashboard.putNumber("PhotonTest_rot", rotationSpeed);
    
      m_drive.arcadeDrive(speed, rotationSpeed);
    } else {
      rotationSpeed = -xboxController.getRightX();
      speed = -xboxController.getLeftY();

      SmartDashboard.putNumber("PhotonTest_speed", speed);
      SmartDashboard.putNumber("PhotonTest_rot", rotationSpeed);
    
      m_drive.arcadeDrive(speed, rotationSpeed);
    }
    
    
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("PhotonFollow done");
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
