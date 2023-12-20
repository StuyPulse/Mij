/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;


import com.pathplanner.lib.auto.PIDConstants;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;


/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
	double DT = 0.02;

	public interface Swerve {
        double WIDTH = Units.inchesToMeters(21);
        double LENGTH = Units.inchesToMeters(21);

		SmartNumber MODULE_VELOCITY_DEADBAND = new SmartNumber("Swerve/Module velocity deadband (m per s)", 0.03);
		SmartNumber MAX_MODULE_SPEED = new SmartNumber("Swerve/Maximum module speed (m per s)", 5.06);
		SmartNumber MAX_MODULE_TURN = new SmartNumber("Swerve/Maximum module turn (rad per s)", 6.28); // TODO: Make this higher

        public interface Turn {
            SmartNumber kP = new SmartNumber("Swerve/Turn/kP", 5.0);
            SmartNumber kI = new SmartNumber("Swerve/Turn/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Turn/kD", 0.05);

            // TODO: FIND THESE VALUES
            SmartNumber kS = new SmartNumber("Swerve/Turn/kS", 0.0);
            SmartNumber kV = new SmartNumber("Swerve/Turn/kV", 0.25);
            SmartNumber kA = new SmartNumber("Swerve/Turn/kA", 0.0);
        }

        public interface Drive {
            SmartNumber kP = new SmartNumber("Swerve/Drive/kP", 0.15083);
            SmartNumber kI = new SmartNumber("Swerve/Drive/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Drive/kD", 0.0); // test with 0.1, seems too high, maybe kA too high

            SmartNumber kS = new SmartNumber("Swerve/Drive/kS", 0.12335);
            SmartNumber kV = new SmartNumber("Swerve/Drive/kV", 2.4132);
            SmartNumber kA = new SmartNumber("Swerve/Drive/kA", 0.15414);
        }

        public interface Motion {
            PIDConstants XY = new PIDConstants(4, 0, 0);
            PIDConstants THETA = new PIDConstants(1, 0, 0);
        }
        
 		public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(71.455078);
                // .plus(Rotation2d.fromDegrees(0));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);
        }

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(43.857422);
                // .plus(Rotation2d.fromDegrees(270));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(204.960938);
                // .plus(Rotation2d.fromDegrees(180));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(149.501953);
                // .plus(Rotation2d.fromDegrees(90));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * -0.5);
        }

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(4);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                double GEAR_RATIO = 1.0 / 6.12;

                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
        }
	}
    
    public interface Driver {
        public interface Drive {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Drive/Deadband", 0.03);

            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.2);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);

            SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Drive/Max Speed", Swerve.MAX_MODULE_SPEED.get());
            SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Drive/Max Accleration", 15);
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.1);

            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.2);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);

            SmartNumber MAX_TELEOP_TURNING = new SmartNumber("Driver Settings/Turn/Max Turning", 6.0);
            SmartNumber MIN_TELEOP_TURNING = new SmartNumber("Driver Settings/Turn/Min Turning", 0.05);

            public interface GyroFeedback {
                SmartBoolean GYRO_FEEDBACK_ENABLED = new SmartBoolean("Driver Settings/Gyro Feedback/Enabled", true);
                SmartNumber P = new SmartNumber("Driver Settings/Gyro Feedback/kP", 6);
                SmartNumber I = new SmartNumber("Driver Settings/Gyro Feedback/kI", 0);
                SmartNumber D = new SmartNumber("Driver Settings/Gyro Feedback/kD", 0.2);
            }
        }

    }

    public static Vector2D vpow(Vector2D vec, double power) {
        return vec.mul(Math.pow(vec.magnitude(), power - 1));
    }
}
