package com.stuypulse.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class SysId extends SubsystemBase {

    protected SysIdRoutine sysIdRoutine;

    public Command quasistaticForward() {
        return sysIdRoutine.quasistatic(Direction.kForward);
    }

    public Command quasistaticReverse() {
        return sysIdRoutine.quasistatic(Direction.kReverse);
    }

    public Command dynamicForward() {
        return sysIdRoutine.dynamic(Direction.kForward);
    }

    public Command dynamicReverse() {
        return sysIdRoutine.dynamic(Direction.kReverse);
    }

    //INTIALIZING ALL DRIVE AND TURN MOTORS
    
    // front left motor
    private final CANSparkMax frontLeftDriveMotor; 
    private final CANSparkMax frontLeftTurnMotor;

    // front right motor
    private final CANSparkMax frontRightDriveMotor;
    private final CANSparkMax frontRightTurnMotor;

    // back left motor
    private final CANSparkMax backLeftDriveMotor;
    private final CANSparkMax backLeftTurnMotor;
    // back right motor
    private final CANSparkMax backRightDriveMotor;
    private final CANSparkMax backRightTurnMotor;    
    
    public SysId(int frontLeftDriveID, int frontRightDriveID, int backLeftDriveID, int backRightDriveID, int frontLeftTurnID, int frontRightTurnID, int backLeftTurnID, int backRightTurnID) {   
        // ASSIGNING ALL DRIVE AND TURN MOTORS
        
        frontLeftDriveMotor = new CANSparkMax(frontLeftDriveID, MotorType.kBrushless);
        frontLeftTurnMotor = new CANSparkMax(frontLeftTurnID, MotorType.kBrushless);

        backLeftDriveMotor = new CANSparkMax(backLeftDriveID, MotorType.kBrushless);
        backLeftTurnMotor = new CANSparkMax(backLeftTurnID, MotorType.kBrushless);

        frontRightDriveMotor = new CANSparkMax(frontRightDriveID, MotorType.kBrushless);
        frontRightTurnMotor = new CANSparkMax(frontRightTurnID, MotorType.kBrushless);

        backRightDriveMotor = new CANSparkMax(backRightDriveID, MotorType.kBrushless);
        backRightTurnMotor = new CANSparkMax(backRightTurnID, MotorType.kBrushless);

        // SETS ALL TURN MOTORS TO ANGLE of 0
        frontLeftTurnMotor.setVoltage(frontLeftTurnID);
        backLeftTurnMotor.setVoltage(backLeftTurnID);
        frontRightTurnMotor.setVoltage(frontRightTurnID);
        backRightTurnMotor.setVoltage(backRightTurnID);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> voltage) -> {
                    frontLeftDriveMotor.setVoltage(voltage.in(Volts));


                    frontRightDriveMotor.setVoltage(voltage.in(Volts));
                    backLeftDriveMotor.setVoltage(voltage.in(Volts));
                    backRightDriveMotor.setVoltage(voltage.in(Volts));
                },
                log -> {
                    log.motor("frontLeftDrive")
                        .voltage(Volts.of(frontLeftDriveMotor.getBusVoltage()))
                        .linearPosition(Meters.of(frontLeftDriveMotor.getEncoder().getPosition()))
                        .linearVelocity(MetersPerSecond.of(frontLeftDriveMotor.getEncoder().getVelocity()));

                    log.motor("frontRightDrive")
                        .voltage(Volts.of(frontRightDriveMotor.getBusVoltage()))
                        .linearPosition(Meters.of(frontRightDriveMotor.getEncoder().getPosition()))
                        .linearVelocity(MetersPerSecond.of(frontRightDriveMotor.getEncoder().getVelocity()));

                    log.motor("backLeftDrive")
                        .voltage(Volts.of(backLeftDriveMotor.getBusVoltage()))
                        .linearPosition(Meters.of(backLeftDriveMotor.getEncoder().getPosition()))
                        .linearVelocity(MetersPerSecond.of(backLeftDriveMotor.getEncoder().getVelocity()));
                    
                    log.motor("backRightDrive")
                        .voltage(Volts.of(backRightDriveMotor.getBusVoltage()))
                        .linearPosition(Meters.of(backRightDriveMotor.getEncoder().getPosition()))
                        .linearVelocity(MetersPerSecond.of(backRightDriveMotor.getEncoder().getVelocity()));
                },
                this
            )
        );
    }
}