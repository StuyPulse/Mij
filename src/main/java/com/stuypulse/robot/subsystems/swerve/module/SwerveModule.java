package com.stuypulse.robot.subsystems.swerve.module;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SwerveModule extends SubsystemBase {

    public abstract String getID();
    public abstract Translation2d getOffset();
    public abstract SwerveModuleState getState();
    public abstract SwerveModulePosition getModulePosition();

    public abstract void setState(SwerveModuleState state);

    public abstract void setTurnVoltage(double voltage);


}
