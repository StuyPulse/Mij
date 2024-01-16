// HAIL NAOWAL RAHMAN III

package com.stuypulse.robot.subsystems.test;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public abstract class TestDrive extends SubsystemBase {

    public abstract Measure<Distance> getPosition();
    public abstract Measure<Velocity<Distance>> getVelocity();
    public abstract void setVoltage(double voltage);
    public abstract Measure<Voltage> getVoltage();
    
    protected SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> voltage) -> {
                setVoltage(voltage.in(Volts));
            },
            log -> {
                log.motor("drive")
                    .voltage(getVoltage())
                    .linearPosition(getPosition())
                    .linearVelocity(getVelocity());
            },
            this
        )
    );

    public Command dynamicForward() {
        return sysIdRoutine.dynamic(Direction.kForward);
    }

    public Command dynamicReverse() {
        return sysIdRoutine.dynamic(Direction.kReverse);
    }

    public Command quasistaticForward() {
        return sysIdRoutine.quasistatic(Direction.kForward);
    }

    public Command quasistaticReverse() {
        return sysIdRoutine.quasistatic(Direction.kReverse);
    }
}
