package com.stuypulse.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

/*
 * 1) list all the components
 * - 4 turn motors, 4 drive motors, 4 absolute encoders, 4 relative encoders
 * -gyro (AHRS)
 * 
 * 2) convert the encoder ticks to degrees/radians
 * 
 * 3) initialze everything 
 * 
 * 4) create swerve drive module class
 * - PID controller for turn module: 
 *      - use setpoint/target method
 * -turn module: angle class for shortest turn
 * - 135 forwards --> 45 degrees backwards
 * - set module speed
 * - contructor: turn encoder, drive encoder
 * - set module states method
 * - get module offsets method 
 * 
 * 5) create kinematics field (takes in 4 offsets)
 * 
 * kinematics.toModuleStates(chassis speeds, x, y, radians counterclockwise)
 * to call on it, use an array 
 * 
 * set chassis speeds 
 * - calls on set module states
 * 
 * .optimize() 
 * 
 * chassisspeeds.fromfieldrelative(speeds towards opponent, speed left, angles we want to turn per sec, current angle)
 */


public class Swerve {
    //creating singleton
    public final static Swerve instance;
    static {
        instance = new Swerve();
    }

    public static Swerve getInstance() {
        return instance;
    }
    
    private final SwerveModule[] modules;  
    private final FieldObject2d[] moduleObjects;
    private final SwerveDriveKinematics kinematics;
    private final AHRS gyro;
    
    public Swerve(SwerveModule... modules) {    
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(getModuleOffsets());
        gyro = new AHRS(SPI.Port.kMXP);
        moduleObjects = new FieldObject2d[modules.length];
    }

    public Translation2d[] getModuleOffsets() {
        var offsets = new Translation2d[modules.length];
        for (int i = 0; i < modules.length; i++) {
            offsets[i] = modules[i].getOffset();
        }
        return offsets;
    }

    public void setChassisSpeeds() {}
    


}
