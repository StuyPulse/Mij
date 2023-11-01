package com.stuypulse.robot.subsystems.swerve;
import com.stuypulse.robot.util.Subsystem;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModule extends Subsystem {

    // data
    private final String id;
    private final Translation2d translationOffset;

    // controllers
    private final Controller driveController;
    private final AngleController turnController;
    
    private SwerveModuleState targetState;

    public SwerveModule(String id, Translation2d translationOffset) {
        super("Swerve/" + id);

        this.id = id;
        this.translationOffset = translationOffset;

        driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD)
                .setOutputFilter(x -> Robot.getMatchState() == Robot.MatchState.TELEOP ? 0 : x)
            .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());

        turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
            .setSetpointFilter(new ARateLimit(Swerve.MAX_MODULE_TURN));

        targetState = new SwerveModuleState();

        registerPeriodicFunc(this::loopModule);
    }

    public final String getID() {
        return id;
    }

    public final Translation2d getOffset() {
        return translationOffset;
    }

    public final void setState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getState().angle);
    }

    public abstract SwerveModuleState getState();
    public abstract SwerveModulePosition getModulePosition();

    protected abstract void setDriveVoltage(double voltage);
    protected abstract void setTurnVoltage(double voltage);

    public void loopModule() {
        setDriveVoltage(driveController.update(
            targetState.speedMetersPerSecond,
            getState().speedMetersPerSecond));
    
        setTurnVoltage(turnController.update(
            Angle.fromRotation2d(targetState.angle),
            Angle.fromRotation2d(getState().angle)));

        putNumber("Drive Voltage", driveController.getOutput());
        putNumber("Turn Voltage", turnController.getOutput());
        
        putNumber("Target Angle", targetState.angle.getDegrees());
        putNumber("Target Speed", targetState.speedMetersPerSecond);
    }

}
