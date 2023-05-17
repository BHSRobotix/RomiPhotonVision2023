// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PhotonTest extends CommandBase {
  PhotonCamera m_camera;
  DriveTrain m_drive;
  PIDController m_turnPID;
  
  /** Creates a new PhotonTest. */
  public PhotonTest(DriveTrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    //camera = new PhotonCamera("photonvision");
    m_camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    m_drive = drive;
    m_turnPID = drive.getTurnPIDController();

    addRequirements(drive);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PhotonTest initialize");
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationSpeed;
    double speed = .5;
    var result = m_camera.getLatestResult();
    SmartDashboard.putBoolean("PhotonTest_targets", result.hasTargets());
    if (result.hasTargets()) {
      rotationSpeed = m_turnPID.calculate(result.getBestTarget().getYaw(), 0);
      rotationSpeed = MathUtil.clamp(rotationSpeed, -.5, .5);
    } else {
      rotationSpeed = 0;
      speed = 0;

    }
    m_drive.arcadeDrive(speed, rotationSpeed);
    
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("PhotonTest done");
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
