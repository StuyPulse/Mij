/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.Arrays;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.stuypulse.robot.subsystems.swerve.SwerveDriveFollowTrajectory;

public class EightFootAuton extends SequentialCommandGroup {
    private static final PathConstraints CONSTRAINTS = new PathConstraints(0.01, 0.01);
    private static final PathPlannerTrajectory PATH = PathPlanner.generatePath(CONSTRAINTS, Arrays.asList(
        new PathPoint(new Translation2d(), new Rotation2d(), new Rotation2d()),
        new PathPoint(new Translation2d(0, 0), new Rotation2d(), new Rotation2d())
    ));
    
    public EightFootAuton() {
        addCommands(
            new SwerveDriveFollowTrajectory(PATH)
                .robotRelative()
                .withStop()
        );
    }

    
}
