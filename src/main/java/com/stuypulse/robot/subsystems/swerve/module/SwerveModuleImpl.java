package com.stuypulse.robot.subsystems.swerve.module;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleImpl extends SwerveModule {

    private final String id;
    private final Rotation2d angleOffset;

    // turn
    private final CANSparkMax turnMotor; 
    private final CANcoder turnEncoder;

    // drive
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder; 

    private final AngleController turnController;
   
    public SwerveModuleImpl(String id, Rotation2d angleOffset, int turnID, int driveID, int encoderID) {
        this.id = id;
        this.angleOffset = angleOffset;
        
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);

        turnMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setIdleMode(IdleMode.kBrake);

        turnEncoder = new CANcoder(encoderID);
        driveEncoder = driveMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        
        turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
            .setSetpointFilter(new ARateLimit(Swerve.MAX_MODULE_TURN))
            .setOutputFilter(x -> -x);
        
        setDriveVoltage(0);
    }

    @Override
    public String getID() {
        return id;
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    @Override
    public double getDriveVoltage() {
        return driveMotor.getBusVoltage();
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnEncoder.getAbsolutePosition().getValueAsDouble()).minus(angleOffset);
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    @Override
    public void periodic() {
        turnMotor.setVoltage(turnController.update(
            Angle.kZero, 
            Angle.fromRotation2d(getAngle())
        ));

        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Drive Voltage", getDriveVoltage());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Turn Voltage", turnController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Angle Error", turnController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Speed", getVelocity());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Raw Encoder Angle", Units.rotationsToDegrees(turnEncoder.getAbsolutePosition().getValueAsDouble()));
    }
}

