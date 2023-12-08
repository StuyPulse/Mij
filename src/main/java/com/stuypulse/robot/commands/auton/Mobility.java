/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.util.SwerveDriveFollowTrajectory;

public class Mobility extends SequentialCommandGroup {
    private static final PathConstraints CONSTRAINTS = new PathConstraints(4, 2);
    
    public Mobility() {
        addCommands(
            new SwerveDriveFollowTrajectory(
                PathPlanner.loadPath("Mobility", CONSTRAINTS)
            ).robotRelative().withStop()
        );
    }

    
}
