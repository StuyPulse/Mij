package com.stuypulse.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings.Swerve.*;
import com.stuypulse.robot.subsystems.swerve.module.SimModule;
import com.stuypulse.robot.subsystems.swerve.module.SwerveModule;
import com.stuypulse.robot.subsystems.swerve.module.SwerveModuleImpl;


public class SysId extends SubsystemBase {

   private final SysIdRoutine sysIdRoutine;

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


   private final SwerveModule[] modules;

   public SysId() {
        if (Robot.isReal()) {
            modules = new SwerveModule[] {
                new SwerveModuleImpl(FrontRight.ID, FrontRight.ABSOLUTE_OFFSET, Ports.Swerve.FrontRight.TURN, Ports.Swerve.FrontRight.DRIVE, Ports.Swerve.FrontRight.ENCODER),
                new SwerveModuleImpl(FrontLeft.ID, FrontLeft.ABSOLUTE_OFFSET, Ports.Swerve.FrontLeft.TURN, Ports.Swerve.FrontLeft.DRIVE, Ports.Swerve.FrontLeft.ENCODER),
                new SwerveModuleImpl(BackLeft.ID, BackLeft.ABSOLUTE_OFFSET, Ports.Swerve.BackLeft.TURN, Ports.Swerve.BackLeft.DRIVE, Ports.Swerve.BackLeft.ENCODER),
                new SwerveModuleImpl(BackRight.ID, BackRight.ABSOLUTE_OFFSET, Ports.Swerve.BackRight.TURN, Ports.Swerve.BackRight.DRIVE, Ports.Swerve.BackRight.ENCODER)
            };
        } else {
            modules = new SwerveModule[] {
                new SimModule(FrontRight.ID),
                new SimModule(FrontLeft.ID),
                new SimModule(BackLeft.ID),
                new SimModule(BackRight.ID)
            };
        }

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> voltage) -> {
                    for (int i = 0; i < 4; i++) {
                        modules[i].setDriveVoltage(voltage.in(Volts));
                    }
                },
                log -> {
                    for (int i = 0; i < 4; i++) {
                        log.motor((modules[i].getID()))
                            .voltage(Volts.of(modules[i].getDriveVoltage()))
                            .linearPosition(Meters.of(modules[i].getModulePosition().distanceMeters))
                            .linearVelocity(MetersPerSecond.of(modules[i].getState().speedMetersPerSecond));
                    }
                },
                this
            )
        );
   }
}

