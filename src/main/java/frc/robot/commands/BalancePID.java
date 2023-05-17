// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class BalancePID extends CommandBase {
  private final DriveTrain m_drive;
  private final double m_distance;
  private final double m_speed;
  
  
  private PIDController m_pid;
  private double m_error;
  private double m_maxOutput;
  private boolean m_forward;
  private boolean m_balance;
  private double m_levelAngle;
  
  /** Creates a new Balance. */
  public BalancePID(double speed, double inches, DriveTrain drive) {
    m_distance = inches;
    m_speed = speed;
    m_maxOutput = .4;
    m_drive = drive;
    m_balance = m_forward = false;
    m_error = 5.0;
    m_pid = m_drive.getBalancePIDController();
    
    addRequirements(drive);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("BalancePID start");
    
    m_pid.reset();
    m_levelAngle = m_drive.getGyroAngleY();
    m_pid.setSetpoint(m_levelAngle);
    m_pid.setTolerance(m_error);
    
    m_forward = true;
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
    
    SmartDashboard.putBoolean("m_forward", m_forward);
    SmartDashboard.putBoolean("m_balance", m_balance);
    SmartDashboard.putNumber("m_levelAngle", m_levelAngle);
    
    SmartDashboard.putData(m_pid);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_drive.getGyroAngleY();
    double output = m_pid.calculate(angle, m_levelAngle);
    
    if (m_forward == true) {
      // Drive forward here. We are not on the ramp, but approaching it.
      m_drive.arcadeDrive(m_speed, 0);
      if (m_pid.atSetpoint() == false) {
        m_balance = true;
        m_forward = false;
      }
      
    } else if (m_balance == true) {  
      // We have gone up the ramp and are trying to balance on it.
      
      // Run pid calc method here to figure out which way and how fast to move
      
      double clampedOutput = MathUtil.clamp(output, -m_maxOutput, m_maxOutput);
      m_drive.arcadeDrive(-clampedOutput, 0);
      
      SmartDashboard.putNumber("output", output);
      SmartDashboard.putNumber("clampedOutput", clampedOutput);
      
      if (m_pid.atSetpoint()) {
        m_balance = true;
        m_forward = false;
      }
    } else {
      // Not driving towards the ramp and we have finished balancing.
      m_drive.arcadeDrive(0, 0);
    }
    
    /* if (Math.abs(angle) > m_levelAngle + m_error) {
      m_balance = true;
      m_forward = false;
    } */
    
    SmartDashboard.putBoolean("m_forward", m_forward);
    SmartDashboard.putBoolean("m_balance", m_balance);
    
    SmartDashboard.putNumber("pidError", m_pid.getPositionError());
    SmartDashboard.putBoolean("pidAtSetpoint", m_pid.atSetpoint());
    
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("BalancePID end interrupted=" + interrupted);
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
