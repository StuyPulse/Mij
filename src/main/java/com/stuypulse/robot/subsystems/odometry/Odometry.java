package com.stuypulse.robot.subsystems.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Odometry extends SubsystemBase {
    private static final Odometry instance;

    static { 
        instance = new OdometryImpl();
    }
    
    public static Odometry getInstance() {
        return instance;
    }

    public abstract void reset(Pose2d pose);
    public abstract Field2d getField();
    public abstract Pose2d getPose();

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }
}