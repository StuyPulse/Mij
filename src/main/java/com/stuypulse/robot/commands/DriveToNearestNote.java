/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands;

import static com.stuypulse.robot.constants.Settings.Vision.NoteDetection.*;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToNearestNote extends Command {

    // Subsystems
    private final SwerveDrive swerve;
    private final Vision vision;

    // Holonomic control
    private final HolonomicController controller;
    private final BStream aligned;

    public DriveToNearestNote(){
        this.swerve = SwerveDrive.getInstance();
        this.vision = Vision.getInstance();
 
        controller = new HolonomicController(
            new PIDController(Translation.P,Translation.I,Translation.D),
            new PIDController(Translation.P, Translation.I, Translation.D),
            new AnglePIDController(Rotation.P, Rotation.I, Rotation.D)
        );

        SmartDashboard.putData("Vision/Controller", controller);

        aligned = BStream.create(this::isAligned).filtered(new BDebounceRC.Rising(DEBOUNCE_TIME));

        addRequirements(swerve);
    }

    private boolean isAligned() {
        return controller.isDone(THRESHOLD_X.get(), THRESHOLD_Y.get(), THRESHOLD_ANGLE.get());
    }

    @Override
    public void execute() {
        double noteDistance  = vision.getDistanceToNote();
        Rotation2d noteRotation = vision.getRotationToNote();

        Pose2d targetPose = new Pose2d(TARGET_NOTE_DISTANCE.get(), 0, new Rotation2d());
        Pose2d currentPose = new Pose2d(noteDistance * noteRotation.getCos(), noteDistance * noteRotation.getSin(), noteRotation);

        swerve.setChassisSpeeds(controller.update(targetPose, currentPose));
        SmartDashboard.putBoolean("Vision/Is Aligned", aligned.get());
    }

    @Override
    public boolean isFinished() {
        return aligned.get();
    }
}