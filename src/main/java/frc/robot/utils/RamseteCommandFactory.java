// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class RamseteCommandFactory {
    
    
    private static Command generateCommand() {
        return null;
    }
    
    private static TrajectoryConfig getTrajectoryConfig() {
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.ksVolts, 
        DriveConstants.kvVoltSecondsPerMeter, 
        DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        DriveConstants.ksMaxVoltage);
        
        TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics)
        .addConstraint(autoVoltageConstraint);
        
        return config;
    }

    public static Trajectory straightTrajectory() {
        TrajectoryConfig config = RamseteCommandFactory.getTrajectoryConfig();

        return TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
        ),
        new Pose2d(Units.inchesToMeters(24), Units.inchesToMeters(0), new Rotation2d(0)),
        config);
    }

    public static Trajectory curveTrajectory() {
        TrajectoryConfig config = RamseteCommandFactory.getTrajectoryConfig();

        return TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
        //new Translation2d(Units.inchesToMeters(6), Units.inchesToMeters(6)),
        //new Translation2d(Units.inchesToMeters(12), Units.inchesToMeters(-6))
        ),
        new Pose2d(Units.inchesToMeters(12), Units.inchesToMeters(12), new Rotation2d(0)),
        config);
    }

    public static Trajectory sCurveTrajectory() {
        TrajectoryConfig config = RamseteCommandFactory.getTrajectoryConfig();

        return TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
            // Drive an S-curve shape
            new Translation2d(Units.inchesToMeters(12), Units.inchesToMeters(12)),
            new Translation2d(Units.inchesToMeters(24), Units.inchesToMeters(-12))
            ),
            new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(0), new Rotation2d(0)),
            config);
    }
    
    public static Trajectory sampleTrajectory() {
        
        TrajectoryConfig config = RamseteCommandFactory.getTrajectoryConfig();
        
        // This trajectory can be modified to suit your purposes
        // Note that all coordinates are in meters, and follow NWU conventions.
        // If you would like to specify coordinates in inches (which might be easier
        // to deal with for the Romi), you can use the Units.inchesToMeters() method
        return TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
        new Translation2d(0.5, 0.25),
        new Translation2d(1.0, -0.25),
        new Translation2d(1.5, 0)
        ),
        new Pose2d(0.0, 0, new Rotation2d(Math.PI)),
        config);
        
    }
    
    /**
    * Generate a trajectory following Ramsete command
    * 
    * This is very similar to the WPILib RamseteCommand example. It uses
    * constants defined in the Constants.java file. These constants were 
    * found empirically by using the frc-characterization tool.
    * 
    * @return A SequentialCommand that sets up and executes a trajectory following Ramsete command
    */
    public static Command generateRamseteCommand(DriveTrain DriveTrain, Trajectory traj) {
        
        DriveTrain.putTrajectoryOnField(traj);
        
        var table = NetworkTableInstance.getDefault().getTable(Debug.DT_DEBUG_TABLE);
        var leftReference = table.getEntry(Debug.DT_L_REF);
        var leftMeasurement = table.getEntry(Debug.DT_L_MEASURE);
        var rightReference = table.getEntry(Debug.DT_R_REF);
        var rightMeasurement = table.getEntry(Debug.DT_R_MEASURE);
        
        PIDController leftPID = new PIDController(DriveConstants.kPDriveVel, 0, 0);
        PIDController rightPID = new PIDController(DriveConstants.kPDriveVel, 0, 0);
        // If debug issues with ramsete/trajectories, use PID with P=0
        //PIDController leftPID = new PIDController(0, 0, 0);
        //PIDController rightPID = new PIDController(0, 0, 0);
        
        RamseteController ramsete = new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta);
        // If debug issues with ramsete/trajectories, disable ramsete controller
        // ramsete.setEnabled(false);
        
        RamseteCommand ramseteCommand = new RamseteCommand(
        traj,
        DriveTrain::getPose,
        ramsete,
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        DriveTrain::getWheelSpeeds,
        leftPID,
        rightPID,
        // DriveTrain::tankDriveVolts,
        (leftVolts, rightVolts) -> {
            DriveTrain.tankDriveVolts(leftVolts, rightVolts);
            
            // Put these values to NT, dashboard.  View them as graphs.  
            // If they do not match/track closely, ya got problems

            leftMeasurement.setNumber(DriveTrain.getWheelSpeeds().leftMetersPerSecond);
            leftReference.setNumber(leftPID.getSetpoint()); 
            rightMeasurement.setNumber(DriveTrain.getWheelSpeeds().rightMetersPerSecond);
            rightReference.setNumber(rightPID.getSetpoint());
        },
        DriveTrain);
        
        DriveTrain.resetOdometry(traj.getInitialPose());
        
        // Set up a sequence of commands
        // First, we want to reset the DriveTrain odometry
        return new InstantCommand(() -> DriveTrain.resetOdometry(traj.getInitialPose()), DriveTrain)
        // next, we run the actual ramsete command
        .andThen(ramseteCommand)
        
        // Finally, we make sure that the robot stops
        .andThen(new InstantCommand(() -> DriveTrain.tankDriveVolts(0, 0), DriveTrain));
    } 
    
    public Command resetOdometry(DriveTrain DriveTrain) {
        return new InstantCommand(() -> DriveTrain.resetOdometry(null), DriveTrain);
    }
}
