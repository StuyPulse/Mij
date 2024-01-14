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
import com.stuypulse.robot.constants.Ports.Swerve.BackLeft;
import com.stuypulse.robot.constants.Ports.Swerve.BackRight;
import com.stuypulse.robot.constants.Ports.Swerve.FrontLeft;
import com.stuypulse.robot.constants.Ports.Swerve.FrontRight;


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


   // KEEP LIST and GET TURN MOTOR AND SET IT TO 0
  
   CANSparkMax[] driveMotors = new CANSparkMax[4];
   CANSparkMax[] turnMotors = new CANSparkMax[4];


   public SysId() {  
    CANSparkMax[] driveMotors = {
        new CANSparkMax(FrontLeft.DRIVE, MotorType.kBrushless),
        new CANSparkMax(FrontRight.DRIVE, MotorType.kBrushless),
        new CANSparkMax(BackLeft.DRIVE, MotorType.kBrushless),
        new CANSparkMax(BackRight.DRIVE, MotorType.kBrushless)
    };

    CANSparkMax[] turnMotors = {
        new CANSparkMax(FrontLeft.TURN, MotorType.kBrushless),
        new CANSparkMax(FrontRight.TURN, MotorType.kBrushless),
        new CANSparkMax(BackLeft.TURN, MotorType.kBrushless),
        new CANSparkMax(BackRight.TURN, MotorType.kBrushless)
    };

       for (int i = 0; i < 4; i++) {
           turnMotors[i].setVoltage(0);
       }


       sysIdRoutine = new SysIdRoutine(
           new SysIdRoutine.Config(),
           new SysIdRoutine.Mechanism(
               (Measure<Voltage> voltage) -> {
                   for (int i = 0; i < 4; i++) {
                       driveMotors[i].setVoltage(voltage.in(Volts));
               }
               },
               log -> {
                   for (int i = 0; i < 4; i++) {
                       log.motor((driveMotors[i].toString()))
                       .voltage(Volts.of(driveMotors[i].getBusVoltage()))
                       .linearPosition(Meters.of(driveMotors[i].getEncoder().getPosition()))
                       .linearVelocity(MetersPerSecond.of(driveMotors[i].getEncoder().getVelocity()));
               }
            },
               this
           )
       );
   }
}

