// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

public class PhotonDriveStraight extends CommandBase {
  PhotonCamera m_camera;
  DriveTrain m_drive;
  PIDController m_turnPID;
  Joystick m_stick;
  CommandXboxController xboxController;
  
  /** Creates a new PhotonTest. */
  public PhotonDriveStraight(DriveTrain drive, Joystick stick) {
    // Use addRequirements() here to declare subsystem dependencies.
    //camera = new PhotonCamera("photonvision");
    m_camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    m_drive = drive;
    m_turnPID = drive.getTurnPIDController();
    m_stick = stick;

    addRequirements(drive);
  }
  public PhotonDriveStraight(DriveTrain drive, CommandXboxController stick) {
      //camera = new PhotonCamera("photonvision");
      m_camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
      m_drive = drive;
      m_turnPID = drive.getTurnPIDController();
      xboxController = stick;
  
      addRequirements(drive);
    }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PhotonDriveStraight initialize");
    SmartDashboard.putData(m_turnPID);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationSpeed;
    //double speed = -m_stick.getY();
    double speed = -xboxController.getLeftY();

    var result = m_camera.getLatestResult();
    SmartDashboard.putBoolean("PhotonTest_targets", result.hasTargets());
    if (result.hasTargets()) {
      rotationSpeed = m_turnPID.calculate(result.getBestTarget().getYaw(), 0);
      rotationSpeed = MathUtil.clamp(rotationSpeed, -.5, .5);
    } else {
      //rotationSpeed = -m_stick.getZ();
      rotationSpeed = -xboxController.getRightX();
    }
    m_drive.arcadeDrive(speed, rotationSpeed);
    
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("PhotonDriveStraight done");
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
