/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
	double DT = 0.02;

	public interface Arm {
		public interface Shoulder {
			//TODO: PLACEHOLDER VALUES
            SmartNumber MAX_SHOULDER_ANGLE = new SmartNumber("Arm/Shoulder/Max Angle (deg)", 0);
            SmartNumber OVER_BUMPER_ANGLE = new SmartNumber("Arm/Shoulder/Over Bumper Angle (deg)", 0);
			
			SmartNumber TELEOP_MAX_VELOCITY = new SmartNumber("Arm/Shoulder/Teleop Max Velocity (deg)", 315);
            SmartNumber TELEOP_MAX_ACCELERATION = new SmartNumber("Arm/Shoulder/Teleop Max Acceleration (deg)", 420);

			int MOTORS = 2;
			Rotation2d ZERO_ANGLE = Rotation2d.fromRotations(0);
			
			public interface PID {
                SmartNumber kP = new SmartNumber("Arm/Shoulder/kP", 0);
                SmartNumber kI = new SmartNumber("Arm/Shoulder/kI", 0);
                SmartNumber kD = new SmartNumber("Arm/Shoulder/kD", 0);
            }

			public interface Feedforward {
                SmartNumber kS = new SmartNumber("Arm/Shoulder/kS", 0);
                SmartNumber kV = new SmartNumber("Arm/Shoulder/kV", 0);
                SmartNumber kA = new SmartNumber("Arm/Shoulder/kA", 0);
				SmartNumber kG = new SmartNumber("Arm/Shoulder/kG", 0);
                SmartNumber kGEmpty = new SmartNumber("Arm/Shoulder/kG Empty", 0);

                SmartNumber kGCube = new SmartNumber("Arm/Shoulder/kG Cube", 0);
                SmartNumber kGCone = new SmartNumber("Arm/Shoulder/kG Cone", 0);
            }
		}

		public interface Wrist {
			int MOTORS = 1;

			Rotation2d ZERO_ANGLE = Rotation2d.fromRotations(0);
			SmartNumber WRIST_SAFE_ANGLE = new SmartNumber("Arm/Wrist/Safe Angle (deg)", 0);
			SmartNumber TELEOP_MAX_VELOCITY = new SmartNumber("Arm/Wrist/Teleop Max Velocity (deg)", 0);
            SmartNumber TELEOP_MAX_ACCELERATION = new SmartNumber("Arm/Wrist/Teleop Max Acceleration (deg)", 0);

			public interface PID {
                SmartNumber kP = new SmartNumber("Arm/Wrist/kP", 0);
                SmartNumber kI = new SmartNumber("Arm/Wrist/kI", 0);
                SmartNumber kD = new SmartNumber("Arm/Wrist/kD", 0);
            }

            public interface Feedforward {
                SmartNumber kS = new SmartNumber("Arm/Wrist/kS", 0);
                SmartNumber kA = new SmartNumber("Arm/Wrist/kA", 0);
                SmartNumber kG = new SmartNumber("Arm/Wrist/kG", 0);
                SmartNumber kV = new SmartNumber("Arm/Wrist/kV", 0);
            }
		}
	}
}
