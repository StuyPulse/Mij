package com.stuypulse.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.MatchState;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleImpl extends SwerveModule {

    // data
    private final String id;
    private final Translation2d translationOffset;
    private final Rotation2d angleOffset;
    private SwerveModuleState targetState;

    // turn
    private final CANSparkMax turnMotor; 
    private final SparkMaxAbsoluteEncoder turnEncoder;

    // drive
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder; 
    
    // controllers
    private final Controller driveController; 
    private final AngleController turnController;
   
    public SwerveModuleImpl(String id, Translation2d translationOffset, Rotation2d angleOffset, int turnID, int driveID) {
        this.id = id;
        this.translationOffset = translationOffset; 
        this.angleOffset = angleOffset;
        
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(turnID, MotorType.kBrushless);

        turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        driveEncoder = driveMotor.getEncoder();
        
        driveController = new PIDController(Drive.kP, Drive.kI, Drive.kP)
                .setOutputFilter(x -> Robot.getMatchState() == MatchState.TELEOP ? 0 : x)
            .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());

        turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kP);
    }

    public Translation2d getOffset() {
        return translationOffset;
    }

    public String getID() {
        return id;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnEncoder.getPosition()).minus(angleOffset);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    public void setState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle());
    }

    @Override
    public void periodic() {
        turnMotor.setVoltage(turnController.update(
            Angle.fromRotation2d(targetState.angle), 
            Angle.fromRotation2d(getAngle()))
        );

        driveMotor.setVoltage(driveController.update(
            targetState.speedMetersPerSecond,
            getVelocity())
        );

        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Drive Voltage", driveController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Turn Voltage", turnController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Target Speed", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Speed", getVelocity());
    }
}

