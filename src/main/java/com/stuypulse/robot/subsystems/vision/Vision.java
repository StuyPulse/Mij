/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.vision;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Vision extends SubsystemBase {

    /** SINGLETON **/
    private static final Vision instance;

    static {
        instance = new VisionImpl();
    }

    public static Vision getInstance() {
        return instance;
    }

    protected Vision() {
    }

    public abstract double getDistanceToNote();
    public abstract Rotation2d getRotationToNote();
}
