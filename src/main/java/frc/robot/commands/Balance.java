// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Balance extends CommandBase {
  private final DriveTrain m_drive;
  private final double m_distance;
  private final double m_speed;
  private double m_error;
  private boolean m_forward;
  private boolean m_balance;
  private double m_levelAngle;
  
  /** Creates a new Balance. */
  public Balance(double speed, double inches, DriveTrain drive) {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
    m_balance = m_forward = false;
    m_error = 5.0;
    
    addRequirements(drive);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Balance start");
    
    m_forward = true;
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
    m_levelAngle = m_drive.getGyroAngleY();

    SmartDashboard.putBoolean("m_forward", m_forward);
    SmartDashboard.putBoolean("m_balance", m_balance);
    SmartDashboard.putNumber("m_levelAngle", m_levelAngle);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_drive.getGyroAngleY();

    if (m_forward == true) {
      m_drive.arcadeDrive(m_speed, 0);
    } else if (m_balance == true) {  
      if (angle > m_levelAngle + m_error) {
        m_drive.arcadeDrive(m_speed, 0);
      } else if (angle < m_levelAngle - m_error) {
        m_drive.arcadeDrive(-m_speed, 0);
      } else {
        //m_drive.arcadeDrive(0, 0);
      }
      if (Math.abs(angle) < m_levelAngle + m_error) {
        m_drive.arcadeDrive(0, 0);
        //m_balance = false;
        m_forward = false;
      }      
    } else {
      m_drive.arcadeDrive(0, 0);
    }

    if (Math.abs(angle) > m_levelAngle + 5.0) {
      m_balance = true;
      m_forward = false;
    }
    SmartDashboard.putBoolean("m_forward", m_forward);
    SmartDashboard.putBoolean("m_balance", m_balance);
    
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Balance end interrupted=" + interrupted);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* double angle = m_drive.getGyroAngleY();
    if (angle < .1 && angle > -.1) {
      return true;
    } */
    return false;
  }
}
