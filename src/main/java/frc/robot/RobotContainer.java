// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.commands.PhotonDriveStraight;
import frc.robot.commands.PhotonFollow;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_DriveTrain = new DriveTrain();
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
  
  // Assumes a gamepad plugged into channnel 0
  //private final Joystick m_controller = new Joystick(0);
  private final CommandXboxController m_controller = new CommandXboxController(0); 
  
  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureAutoRoutines();
  }
  
  /**
  * Use this method to define your button->command mappings. Buttons can be created by
  * instantiating a {@link GenericHID} or one of its subclasses ({@link
    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
      * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
      */
      private void configureButtonBindings() {
        // Default command is arcade drive. This will run unless another command
        // is scheduled over it.
        m_DriveTrain.setDefaultCommand(getArcadeDriveCommand());
        
        // Example of how to use the onboard IO
        Trigger onboardButtonA = new Trigger(m_onboardIO::getButtonAPressed);
        onboardButtonA
        .onTrue(new PrintCommand("Button A Pressed"))
        .onFalse(new PrintCommand("Button A Released"));
        
        m_controller.x().onTrue(new PrintCommand("button X onTrue"));
        m_controller.y().onTrue(new PrintCommand("button Y onTrue"));
        m_controller.a().onTrue(new PrintCommand("button A onTrue"));
        m_controller.b().onTrue(new PrintCommand("button B onTrue"));

        // m_controller.x().toggleOnTrue(RamseteCommandFactory.generateRamseteCommand(m_DriveTrain, RamseteCommandFactory.sCurveTrajectory()));
        // m_controller.y().toggleOnTrue(RamseteCommandFactory.generateRamseteCommand(m_DriveTrain, RamseteCommandFactory.straightTrajectory()));

        // m_controller.a().toggleOnTrue(PPCommandFactory.followTrajectoryCommand(m_DriveTrain, PPCommandFactory.readTrajectory(), true));
        //m_controller.y().toggleOnTrue(RamseteCommandFactory.generateRamseteCommand(m_DriveTrain, RamseteCommandFactory.straightTrajectory()));

        // new JoystickButton(m_controller, Button.kB.value).toggleOnTrue(new PhotonDriveStraight(m_DriveTrain, m_controller));
        // new JoystickButton(m_controller, Button.kA.value).toggleOnTrue(new PhotonDriveDistance(m_DriveTrain, m_controller));
        m_controller.a().toggleOnTrue(new PhotonFollow(m_DriveTrain, m_controller));
        m_controller.y().toggleOnTrue(new PhotonDriveStraight(m_DriveTrain, m_controller));
      }

      public void configureAutoRoutines() {
        // Setup SmartDashboard options
        m_chooser.setDefaultOption("Auto Routine Distance", new AutonomousDistance(m_DriveTrain));
        //m_chooser.setDefaultOption("Auto Routine Balance", new Balance(.5, 1, m_DriveTrain));
        m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_DriveTrain));
        SmartDashboard.putData(m_chooser);
      }
      
      /**
      * Use this to pass the autonomous command to the main {@link Robot} class.
      *
      * @return the command to run in autonomous
      */
      public Command getAutonomousCommand() {
        return m_chooser.getSelected();
      }
      
      /**
      * Use this to pass the teleop command to the main {@link Robot} class.
      *
      * @return the command to run in teleop
      */
      public Command getArcadeDriveCommand() {
        return new ArcadeDrive(
        m_DriveTrain, () -> -m_controller.getRawAxis(1), () -> -m_controller.getRawAxis(2));
      }
    }
    