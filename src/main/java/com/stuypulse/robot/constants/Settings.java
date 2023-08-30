/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
	double DT = 0.02;
    public interface Intake {
        SmartNumber STALL_TIME = new SmartNumber("Intake/Stall Time (Rising)", 0.05);
        SmartNumber CUBE_STALL_CURRENT = new SmartNumber("Cube Intake/Stall Current", 35);
        SmartNumber CONE_STALL_CURRENT = new SmartNumber("Cone Intake/Stall Current", 35);

    }
}
