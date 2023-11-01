package com.stuypulse.robot.subsystems.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class SimModule extends SwerveModule {

    private static LinearSystem<N2, N1, N2> identifyVelocityPositionSystem(double kV, double kA) {
        if (kV <= 0.0) {
            throw new IllegalArgumentException("Kv must be greater than zero.");
        }
        if (kA <= 0.0) {
            throw new IllegalArgumentException("Ka must be greater than zero.");
        }

        return new LinearSystem<N2, N1, N2>(
            Matrix.mat(Nat.N2(), Nat.N2()).fill(0.0, 1.0, 0.0, -kV / kA),
            Matrix.mat(Nat.N2(), Nat.N1()).fill(0.0, 1.0 / kA),
            Matrix.mat(Nat.N2(), Nat.N2()).fill(1.0, 0.0, 0.0, 1.0),
            Matrix.mat(Nat.N2(), Nat.N1()).fill(0.0, 0.0)
        );
    }

    // sim
    private final LinearSystemSim<N2, N1, N2> driveSim;
    private final LinearSystemSim<N2, N1, N1> turnSim;

    public SimModule(String id, Translation2d translationOffset) {
        super(id, translationOffset);

        driveSim = new LinearSystemSim<>(identifyVelocityPositionSystem(Drive.kV.get(), Drive.kA.get()));
        turnSim = new LinearSystemSim<N2, N1, N1>(LinearSystemId.identifyPositionSystem(Turn.kV.get(), Turn.kA.get()));

        registerPeriodicFunc(this::loopSim);
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    private double getDistance() {
        return driveSim.getOutput(0);
    }

    private double getVelocity() {
        return driveSim.getOutput(1);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRadians(turnSim.getOutput(0));
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }

    @Override
    protected void setDriveVoltage(double voltage) {
        driveSim.setInput(voltage);
    }

    @Override
    protected void setTurnVoltage(double voltage) {
        turnSim.setInput(voltage);
    }

    public void loopSim() {
        putNumber("Angle", getAngle().getDegrees());
        putNumber("Speed", getVelocity());
    }

    @Override
    public void simulationPeriodic() {
        driveSim.update(Settings.DT);
        turnSim.update(Settings.DT);

       // understand how it works
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
            turnSim.getCurrentDrawAmps() + driveSim.getCurrentDrawAmps()
        ));
    }
}
