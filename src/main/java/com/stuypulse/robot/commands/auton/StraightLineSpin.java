/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.swerve.SwerveDriveFollowTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StraightLineSpin extends SequentialCommandGroup {
    private static final PathConstraints CONSTRAINTS = new PathConstraints(2, 2);
    
    public StraightLineSpin() {
        addCommands(
            new SwerveDriveFollowTrajectory(
                PathPlanner.loadPath("Mobility", CONSTRAINTS)
            ).robotRelative().withStop(),
            new InstantCommand(() -> SwerveDrive.getInstance().setTurnVoltage(1))
        );
    }




    
}
