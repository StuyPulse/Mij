/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.auton;

import java.util.Arrays;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.stuypulse.robot.util.SwerveDriveFollowTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class EightFootAuton extends SequentialCommandGroup {
    private static final PathConstraints CONSTRAINTS = new PathConstraints(0.01, 0.01, 0.01, 0.01);
    private static final PathPlannerPath PATH = PathPlannerPath.fromPathPoints(
        Arrays.asList(
            new PathPoint(new Translation2d(0, 0)),
            new PathPoint(new Translation2d(2.4384, 0.0))),
        CONSTRAINTS, new GoalEndState(0, new Rotation2d()));
    
    public EightFootAuton() {
        addCommands(
            new SwerveDriveFollowTrajectory(PATH)
                .robotRelative()
                .withStop()
        );
    }

    
}
