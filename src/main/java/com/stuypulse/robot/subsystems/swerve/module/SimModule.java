package com.stuypulse.robot.subsystems.swerve.module;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimModule extends SwerveModule {

    private static LinearSystem<N2, N1, N2> identifyVelocityPositionSystem(double kV, double kA) {
        if (kV <= 0.0) {
            throw new IllegalArgumentException("Kv must be greater than zero.");
        }
        if (kA <= 0.0) {
            throw new IllegalArgumentException("Ka must be greater than zero.");
        }

        return new LinearSystem<N2, N1, N2>(
            MatBuilder.fill(Nat.N2(), Nat.N2(), 0.0, 1.0, 0.0, -kV / kA),
            MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 1.0 / kA),
            MatBuilder.fill(Nat.N2(), Nat.N2(), 1.0, 0.0, 0.0, 1.0),
            MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 0.0)
        );
    }

    private final String id;
    private AngleController turnController;

    private LinearSystemSim<N2, N1, N2> driveSim;
    private LinearSystemSim<N2, N1, N1> turnSim;

    private double voltage;

    public SimModule(String id) {
        this.id = id;

        turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
            .setSetpointFilter(new ARateLimit(Swerve.MAX_MODULE_TURN));

        driveSim = new LinearSystemSim<>(identifyVelocityPositionSystem(Drive.kV.get(), Drive.kA.get()));
        turnSim = new LinearSystemSim<N2, N1, N1>(LinearSystemId.identifyPositionSystem(Turn.kV.get(), Turn.kA.get()));
    }

    @Override
    public String getID() {
        return id;
    }

    @Override
    public void setDriveVoltage(double voltage) {
        this.voltage = voltage;
        driveSim.setInput(voltage); 
    }

    @Override
    public double getDriveVoltage() {
        return voltage;
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public double getDistance() {
        return driveSim.getOutput(0);
    }

    public double getVelocity() {
        return driveSim.getOutput(1);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(turnSim.getOutput(0));
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }
        
    @Override
    public void periodic() {
        turnController.update(
            Angle.kZero,
            Angle.fromRotation2d(getAngle())
        );

        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Turn Voltage", turnController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Drive Voltage", getDriveVoltage());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Speed", getVelocity());
    }

    @Override
    public void simulationPeriodic() {
        driveSim.update(Settings.DT);
        turnSim.update(Settings.DT);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
            turnSim.getCurrentDrawAmps() + driveSim.getCurrentDrawAmps()
        ));
    }
}
