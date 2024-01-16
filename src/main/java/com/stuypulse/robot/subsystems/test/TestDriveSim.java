package com.stuypulse.robot.subsystems.test;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestDriveSim extends TestDrive {
    // This is used to simulate holding/using the voltagegs 
    private static LinearSystem<N2, N1, N2> identifyVelocityPositionSystem(double kV, double kA) {
        if (kV <= 0.0) {
            throw new IllegalArgumentException("kV must be greater than zero.");
        }
        if (kA <= 0.0) {
            throw new IllegalArgumentException("kA must be greater than zero.");
        }

        return new LinearSystem<N2, N1, N2>(
            MatBuilder.fill(Nat.N2(), Nat.N2(), 0.0, 1.0, 0.0, -kV / kA),
            MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 1.0 / kA),
            MatBuilder.fill(Nat.N2(), Nat.N2(), 1.0, 0.0, 0.0, 1.0),
            MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 0.0)
        );
    }

    private LinearSystemSim<N2, N1, N2> driveSim;
    private double voltage;

    public TestDriveSim() {
        driveSim = new LinearSystemSim<>(identifyVelocityPositionSystem(1, 1));
    }

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
        driveSim.setInput(voltage);
    }


    @Override
    public Measure<Distance> getPosition() {
        return Meters.of(driveSim.getOutput(0));
    }

    @Override
    public Measure<Velocity<Distance>> getVelocity() {
        return MetersPerSecond.of(driveSim.getOutput(1));
    }

    @Override
    public void periodic() {
        driveSim.update(Settings.DT);

        SmartDashboard.putNumber("Test Drive Sim/Voltage", voltage);
        SmartDashboard.putNumber("Test Drive Sim/Position", getPosition().magnitude());
        SmartDashboard.putNumber("Test Drive Sim/Velocity", getVelocity().magnitude());
    }

    public Measure<Voltage> getVoltage() {
        return Volts.of(voltage);
    }
}
