/************************ PROJECT JIM *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.ReplanningConfig;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveFollowTrajectory extends FollowPathCommand {

	public static HashMap<String, PathPlannerTrajectory> getSeparatedPaths(List<PathPlannerTrajectory> paths, String... names) {
		if (paths.size() != names.length)
			throw new IllegalArgumentException("Invalid number of path names given to FollowTrajectory.getSeparatedPaths");

		HashMap<String, PathPlannerTrajectory> map = new HashMap<String, PathPlannerTrajectory>();

		for (int i = 0; i < names.length; i++) {
			map.put(names[i], paths.get(i));
		}

		return map;
	}

	private boolean robotRelative;
	private boolean shouldStop;
	private PathPlannerTrajectory path;
	private HashMap<String, Command> events;

	private FieldObject2d trajectory;

	public SwerveDriveFollowTrajectory(PathPlannerPath path) {

		super(
			path,
			Odometry.getInstance()::getPose,
			SwerveDrive.getInstance()::getChassisSpeeds,
			SwerveDrive.getInstance()::setChassisSpeeds,
			new PPHolonomicDriveController(Motion.XY, Motion.THETA, Settings.Swerve.MAX_MODULE_SPEED.get(), Settings.Swerve.WIDTH),
			new ReplanningConfig(),
			() -> {
				

				var alliance = DriverStation.getAlliance();
				if (alliance.isPresent()) {
					return alliance.get() == DriverStation.Alliance.Red;
				}
				return false;
			},
			SwerveDrive.getInstance()

		);

		robotRelative = false;
		trajectory = Odometry.getInstance().getField().getObject("Trajectory");
		this.path = new PathPlannerTrajectory(path, SwerveDrive.getInstance().getChassisSpeeds(), new Rotation2d());
		events = new HashMap<String, Command>();
		shouldStop = false;
	}

	public SwerveDriveFollowTrajectory withStop() {
		shouldStop = true;
		return this;
	}

	public SwerveDriveFollowTrajectory robotRelative() {
		robotRelative = true;
		return this;
	}

	public SwerveDriveFollowTrajectory fieldRelative() {
		robotRelative = false;
		return this;
	}

	public SwerveDriveFollowTrajectory addEvent(String name, Command command) {
		events.put(name, command);
		return this;
	}

	// FINISHES AT END OF PATH FOLLOWING, NOT AFTER ALL EVENTS DONE
	// public FollowPathWithEvents withEvents() {
	// 	return new FollowPathWithEvents(
	// 		this,
	// 		path.getMarkers(),
	// 		events
	// 	);
	// }

	@Override
	public void initialize() {
		if (robotRelative) {
			PathPlannerTrajectory.State initialState = 
				path.getInitialState();

			Odometry.getInstance().reset(new Pose2d(
				initialState.positionMeters,
				initialState.targetHolonomicRotation
			));
		}

		// trajectory.setTrajectory(PathPlannerTrajectory.transformTrajectoryForAlliance(path, DriverStation.getAlliance()));

		super.initialize();
	}

	@Override
	public void end(boolean interrupted) {
		if (shouldStop) {
			SwerveDrive.getInstance().setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
		}
		trajectory.setPose(
			new Pose2d(Double.NaN, Double.NaN, new Rotation2d(Double.NaN))
		);
	}

}