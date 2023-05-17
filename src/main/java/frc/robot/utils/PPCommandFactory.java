// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class PPCommandFactory {
    
    public static PathPlannerTrajectory readTrajectory() {
        return PathPlanner.loadPath("Straight", new PathConstraints(4, 3));
        
    }
    
    public static Command followTrajectoryCommand(DriveTrain dt, PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
        new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                dt.resetOdometry(traj.getInitialPose());
            }
        }),
        new PPRamseteCommand(
        traj, 
        dt::getPose, // Pose supplier
        new RamseteController(),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, 
        DriveConstants.kvVoltSecondsPerMeter, 
        DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        dt::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
        new PIDController(DriveConstants.kPDriveVel, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        new PIDController(DriveConstants.kPDriveVel, 0, 0), // Right controller (usually the same values as left controller)
        (leftVolts, rightVolts) -> {
            dt.tankDriveVolts(leftVolts, rightVolts);
        },
        //dt::tankDriveVolts,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        dt // Requires this drive subsystem
        )
        );
    }
}
