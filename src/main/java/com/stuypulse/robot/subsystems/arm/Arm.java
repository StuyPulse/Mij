package com.stuypulse.robot.subsystems.arm;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.util.ArmEncoderFeedforward;
import com.stuypulse.robot.util.RichieMotionProfile;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase {
    
    private static final Arm instance;

    static {
        if (RobotBase.isReal()) {
            instance = new ArmImpl();
        } else {
            instance = null; //TODO: Look over this later
        }
    }

    public static Arm getInstance() {
        return instance;
    }
    
    // ARM VARIABLES
    private final SmartNumber shoulderTargetDegrees;
    private final SmartNumber wristTargetDegrees;

    // Voltage overrides (used when present)
    private Optional<Double> wristVoltageOverride;
    private Optional<Double> shoulderVoltageOverride;

    private final SmartNumber shoulderMaxVelocity;
    private final SmartNumber shoulderMaxAcceleration;

    private final SmartNumber wristMaxVelocity;
    private final SmartNumber wristMaxAcceleration;
    
    private final Controller shoulderController;
    private final Controller wristController;

    private final RichieMotionProfile shoulderMotionProfile;
    private final RichieMotionProfile wristMotionProfile;

    protected Arm() {
        shoulderTargetDegrees = new SmartNumber("Arm/Shoulder/Target Angle (deg)", 0);
        wristTargetDegrees = new SmartNumber("Arm/Wrist/Target Angle (deg)", 0);

        shoulderMaxVelocity = new SmartNumber(
            "Arm/Shoulder/Max Velocity",
            Shoulder.TELEOP_MAX_VELOCITY.doubleValue());
        shoulderMaxAcceleration = new SmartNumber(
            "Arm/Shoulder/Max Acceleration",
            Shoulder.TELEOP_MAX_ACCELERATION.doubleValue());

        wristMaxVelocity = new SmartNumber(
            "Arm/Wrist/Max Velocity",
            Wrist.TELEOP_MAX_VELOCITY.doubleValue());
        wristMaxAcceleration = new SmartNumber(
            "Arm/Wrist/Max Acceleration",
            Wrist.TELEOP_MAX_ACCELERATION.doubleValue());

        wristVoltageOverride = Optional.empty();
        shoulderVoltageOverride = Optional.empty();

        shoulderMotionProfile = new RichieMotionProfile(shoulderMaxVelocity, shoulderMaxAcceleration);
        wristMotionProfile = new RichieMotionProfile(wristMaxVelocity, wristMaxAcceleration);

        shoulderController = new MotorFeedforward(Shoulder.Feedforward.kS, Shoulder.Feedforward.kV, Shoulder.Feedforward.kA).position()
            .add(new ArmEncoderFeedforward(Shoulder.Feedforward.kG))
            .add(new PIDController(Shoulder.PID.kP, Shoulder.PID.kI, Shoulder.PID.kD))
            .setSetpointFilter(shoulderMotionProfile)
            .setOutputFilter(x -> shoulderVoltageOverride.orElse(x));
        
        wristController = new MotorFeedforward(Wrist.Feedforward.kS, Wrist.Feedforward.kV, Wrist.Feedforward.kA).position()
            .add(new PIDController(Wrist.PID.kP, Wrist.PID.kI, Wrist.PID.kD))
            .setSetpointFilter(wristMotionProfile)
            .setOutputFilter(x -> wristVoltageOverride.orElse(x));
    }

    // SETTERS
    public final void setShoulderTargetAngle(Rotation2d angle) {
        shoulderVoltageOverride = Optional.empty();
        shoulderTargetDegrees.set(angle.getDegrees());
    }

    public final void setWristTargetAngle(Rotation2d angle) {
        wristVoltageOverride = Optional.empty();
        wristTargetDegrees.set(angle.getDegrees());
    }

    public abstract void setCoast(boolean wristCoast, boolean shoulderCoast);

    public final void setShoulderConstraints(Number velocity, Number acceleration) {
        shoulderMaxVelocity.set(velocity);
        shoulderMaxAcceleration.set(acceleration);
    } 
    public final void setWristConstraints(Number velocity, Number acceleration) {
        wristMaxVelocity.set(velocity);
        wristMaxAcceleration.set(acceleration);
    }

    public void setShoulderVoltage(double voltage) {
        shoulderVoltageOverride = Optional.of(voltage);
    }
    public void setWristVoltage(double voltage) {
        wristVoltageOverride = Optional.of(voltage);
    }
    
    // Feed a voltage to the hardware layer
    protected abstract void setShoulderVoltageImpl(double voltage);
    protected abstract void setWristVoltageImpl(double voltage);

    protected abstract double getShoulderVelocityRadiansPerSecond();
    protected abstract double getWristVelocityRadiansPerSecond();

    // GETTERS
    public final Rotation2d getShoulderTargetAngle() {
        return Rotation2d.fromDegrees(shoulderTargetDegrees.get());
    }

    public final Rotation2d getWristTargetAngle() {
        return Rotation2d.fromDegrees(wristTargetDegrees.get());
    }

    public abstract Rotation2d getShoulderAngle();

    protected abstract Rotation2d getRelativeWristAngle();

    public final Rotation2d getWristAngle() {
        return getShoulderAngle().plus(getRelativeWristAngle());
    }

    // isAt
    public final boolean isShoulderAtTarget(double epsilonDegrees) {
        return Math.abs(getShoulderTargetAngle().minus(getShoulderAngle()).getDegrees()) < epsilonDegrees;
    }

    public final boolean isWristAtTarget(double epsilonDegrees) {
        return Math.abs(getWristTargetAngle().minus(getWristAngle()).getDegrees()) < epsilonDegrees;
    }

    @Override
    public final void periodic() {
        // Run control loops on validated target angles
        shoulderController.update(
            getShoulderTargetAngle().getRadians(),
            getShoulderAngle().getRadians()
        );

        wristController.update(
            getWristTargetAngle().getRadians(),
            getWristAngle().getRadians()
        );

        setShoulderVoltageImpl(shoulderController.getOutput()); 
        setWristVoltageImpl(wristController.getOutput());
          
        SmartDashboard.putNumber("Arm/Shoulder/Angle (deg)", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Shoulder/Setpoint (deg)", Units.radiansToDegrees(shoulderController.getSetpoint()));
        SmartDashboard.putNumber("Arm/Shoulder/Error (deg)", Units.radiansToDegrees(shoulderController.getError()));
        SmartDashboard.putNumber("Arm/Shoulder/Output (V)", shoulderController.getOutput());
        SmartDashboard.putNumber("Arm/Shoulder/Velocity (deg per s)", Units.radiansToDegrees(getShoulderVelocityRadiansPerSecond()));

        SmartDashboard.putNumber("Arm/Wrist/Angle (deg)", getWristAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Relative Angle (deg)", getRelativeWristAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Wrist/Setpoint (deg)", Units.radiansToDegrees(wristController.getSetpoint()));
        SmartDashboard.putNumber("Arm/Wrist/Error (deg)", Units.radiansToDegrees(wristController.getError()));
        SmartDashboard.putNumber("Arm/Wrist/Output (V)", wristController.getOutput());
        SmartDashboard.putNumber("Arm/Wrist/Velocity (deg per s)", Units.radiansToDegrees(getWristVelocityRadiansPerSecond()));

        periodicallyCalled();
    }

    protected void periodicallyCalled() {}
    
}