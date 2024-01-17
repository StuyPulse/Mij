/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import static com.stuypulse.robot.constants.Settings.Vision.Limelight.LIMELIGHTS;
import static com.stuypulse.robot.constants.Settings.Vision.Limelight.POSITIONS;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight {
    private final String tableName;
    private final IntegerEntry idEntry;
    
    private final DoubleEntry txEntry;
    private final DoubleEntry tyEntry;
    private final IntegerEntry tvEntry;
    private final IntegerEntry pipelineEntry;

    private int limelightId;
    private final Pose3d robotRelativePose;


    public Limelight(String tableName, Pose3d robotRelativePose) {
        this.tableName = tableName;
        this.robotRelativePose = robotRelativePose;

        for(int i = 0; i < LIMELIGHTS.length; i++) {
            if(LIMELIGHTS[i].equals(tableName)) {
                limelightId = i;
            }
        }

        NetworkTable limelight = NetworkTableInstance.getDefault().getTable(tableName);

        idEntry = limelight.getIntegerTopic("tid").getEntry(0);

        txEntry = limelight.getDoubleTopic("tx").getEntry(0.0);
        tyEntry = limelight.getDoubleTopic("ty").getEntry(0.0);
        tvEntry = limelight.getIntegerTopic("tv").getEntry(0);

        pipelineEntry = limelight.getIntegerTopic("pipeline").getEntry(0);
    }

    public String getTableName() {
        return tableName;
    }

    public boolean hasNoteData() {
        return tvEntry.get() == 1;
    }

    public double getXAngle() {
        if(hasNoteData()) {
            return Double.NaN;
        }

        return txEntry.get() + Units.radiansToDegrees(POSITIONS[limelightId].getRotation().getZ());
    }

    public double getYAngle() {
        if(!hasNoteData()) {
            return Double.NaN;
        }

        return tyEntry.get() + Units.radiansToDegrees(POSITIONS[limelightId].getRotation().getY());
    }

    public double getDistanceToNote() {
        if(!hasNoteData()) {
            return Double.NaN;
        }

        Rotation2d yRotation =  Rotation2d.fromDegrees(getYAngle());
        return POSITIONS[limelightId].getZ() / yRotation.getTan() + POSITIONS[limelightId].getX();
    }
}