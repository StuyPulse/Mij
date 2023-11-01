package com.stuypulse.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleImpl extends SwerveModule {

    // data
    private final Rotation2d angleOffset;

    // turn
    private final CANSparkMax turnMotor; 
    private final SparkMaxAbsoluteEncoder turnEncoder;

    // drive
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder; 
   
    public SwerveModuleImpl(String id, Translation2d translationOffset, Rotation2d angleOffset, int turnID, int driveID) {
        super(id, translationOffset);
        
        this.angleOffset = angleOffset;
        
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(turnID, MotorType.kBrushless);

        turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        driveEncoder = driveMotor.getEncoder();
        
        registerPeriodicFunc(this::loopImpl);
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    private double getVelocity() {
        return driveEncoder.getVelocity();
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnEncoder.getPosition()).minus(angleOffset);
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    @Override
    protected void setDriveVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    @Override
    protected void setTurnVoltage(double voltage) {
        turnMotor.setVoltage(voltage);
    }

    public void loopImpl() {
        putNumber("Angle", getAngle().getDegrees());
        putNumber("Speed", getVelocity());
    }
}

