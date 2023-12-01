package com.stuypulse.robot.subsystems.swerve;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;

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
            Matrix.mat(Nat.N2(), Nat.N2()).fill(0.0, 1.0, 0.0, -kV / kA),
            Matrix.mat(Nat.N2(), Nat.N1()).fill(0.0, 1.0 / kA),
            Matrix.mat(Nat.N2(), Nat.N2()).fill(1.0, 0.0, 0.0, 1.0),
            Matrix.mat(Nat.N2(), Nat.N1()).fill(0.0, 0.0)
        );
    }

    private final String id;
    private final Translation2d translationOffset;
    private SwerveModuleState targetState;

    private Controller driveController;
    private AngleController turnController;

    private LinearSystemSim<N2, N1, N2> driveSim;
    private LinearSystemSim<N2, N1, N1> turnSim;

    public SimModule(String id, Translation2d translationOffset) {
        this.id = id;
        this.translationOffset = translationOffset;         

        driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD)
                .setOutputFilter(x -> Robot.getMatchState() == Robot.MatchState.TELEOP ? 0 : x)
            .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());

        turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
            .setSetpointFilter(new ARateLimit(Swerve.MAX_MODULE_TURN));

        driveSim = new LinearSystemSim<>(identifyVelocityPositionSystem(Drive.kV.get(), Drive.kA.get()));
        turnSim = new LinearSystemSim<N2, N1, N1>(LinearSystemId.identifyPositionSystem(Turn.kV.get(), Turn.kA.get()));

        targetState = new SwerveModuleState();
    }

    public String getID() {
        return id;
    }

    public Translation2d getOffset() {
        return translationOffset;
    }

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

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }
        
    public void setState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle());
    }

    @Override
    public void periodic() {
        driveController.update(
            targetState.speedMetersPerSecond,
            getVelocity()
        );

        turnController.update(
            Angle.fromRotation2d(targetState.angle),
            Angle.fromRotation2d(getAngle())
        );

        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Drive Voltage", driveController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Turn Voltage", turnController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Target Speed", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Speed", getVelocity());
    }

    @Override
    public void simulationPeriodic() {
        driveSim.setInput(driveController.getOutput());
        driveSim.update(Settings.DT);

        turnSim.setInput(turnController.getOutput());
        turnSim.update(Settings.DT);

       // understand how it works
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
            turnSim.getCurrentDrawAmps() + driveSim.getCurrentDrawAmps()
        ));
    }
}
