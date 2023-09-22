package com.stuypulse.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    // data
    private final String id;
    private SwerveModuleState targetState;
    private Translation2d translationOffset;
    private Rotation2d angleOffset;
    // turn
    private CANSparkMax turnMotor; 
    private SparkMaxAbsoluteEncoder turnEncoder;

    // drive
    private CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder; 
 
    // controllers
    private Controller driveController; 
    private AngleController turnController;
   
    public SwerveModule(String id, Translation2d translationOffset, Rotation2d angleOffset, int turnID, int driveID) {
        this.id = id;
        this.translationOffset = translationOffset; 
        
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(turnID, MotorType.kBrushless);

        turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        driveEncoder = driveMotor.getEncoder();
        
        driveController = new PIDController(0, 0, 0) // make these constants
            .add(new MotorFeedforward(0, 0, 0).velocity());

        turnController = new AnglePIDController(0, 0, 0);


           
        
   }

   /*
    *  4) create swerve drive module class
    * - PID controller for turn module: 
    *      - use setpoint/target method
    * -turn module: angle class for shortest turn
    * - 135 forwards --> 45 degrees backwards
    * - set module speed
    * - contructor: turn encoder, drive encoder
    * - set module states method
    * - get module offsets method 
    * Methods: 
        -get ID (string)
        -get offset (translation 2d)
        - getState (Swerve Module State)
        -get velocity from drive encoder.getVelocity (double)
        - get angle from absolute encoder (rotation 2d) 
        -set target state optimized (void)
        -getModule position from drive encoder(swerveModule position)
        -periodic: 
            - turn motor and drive motor set volutage (controller. update(targetState.angle, get angle))
            - drive motor: get value from targetstate, get velocity   
            */
    
    public Translation2d getOffset() {
        return translationOffset;
    }

    public String getID(){
        return id;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public double getVelocity(){
        return driveEncoder.getVelocity();
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRotations(turnEncoder.getPosition()).minus(angleOffset);
    }
    
    public void setState(SwerveModuleState state){
        targetState = SwerveModuleState.optimize(state, getAngle());
    }

    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    public void periodic(){
        turnMotor.setVoltage(turnController.update(
            Angle.fromRotation2d(targetState.angle), 
            Angle.fromRotation2d(getAngle()) ));
        driveMotor.setVoltage(driveController.update(
            targetState.speedMetersPerSecond,
            getVelocity()));

        
    }

}

