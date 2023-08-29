package com.stuypulse.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.stuypulse.robot.constants.Ports.Arm.*;
import static com.stuypulse.robot.constants.Ports.Wrist.*;

public class ArmImpl extends Arm {

	private final CANSparkMax shoulderLeft;
    private final CANSparkMax shoulderRight;
    private final CANSparkMax wrist;
	
	private final AbsoluteEncoder shoulderEncoder;
    private final AbsoluteEncoder wristEncoder;

    protected ArmImpl() {
        shoulderLeft = new CANSparkMax(LEFT, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(RIGHT, MotorType.kBrushless);
        wrist = new CANSparkMax(MOTOR, MotorType.kBrushless);

        shoulderEncoder = shoulderRight.getAbsoluteEncoder(Type.kDutyCycle); //TODO: check if this is the right motor to check
        wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);

    }

	@Override
	protected void setShoulderVoltageImpl(double voltage) {
		shoulderLeft.setVoltage(voltage);
        shoulderRight.setVoltage(voltage);
	}

	@Override
	protected void setWristVoltageImpl(double voltage) {
        wrist.setVoltage(voltage);
	}

	@Override
	public Rotation2d getShoulderAngle() {
        return Rotation2d.fromRotations(shoulderEncoder.getPosition()).minus(Settings.Arm.Shoulder.ZERO_ANGLE);
	}

	@Override
	protected Rotation2d getRelativeWristAngle() {
        return Rotation2d.fromRotations(wristEncoder.getPosition()).minus(Settings.Arm.Wrist.ZERO_ANGLE);
	}

    @Override
	public void setCoast(boolean wristCoast, boolean shoulderCoast) {
        shoulderLeft.setIdleMode(shoulderCoast ? IdleMode.kCoast : IdleMode.kBrake);
        shoulderRight.setIdleMode(shoulderCoast ? IdleMode.kCoast : IdleMode.kBrake);
        wrist.setIdleMode(wristCoast ? IdleMode.kCoast : IdleMode.kBrake);
	}

	@Override
	protected double getShoulderVelocityRadiansPerSecond() {
        return Units.rotationsToRadians(shoulderEncoder.getVelocity());
	}

	@Override
	protected double getWristVelocityRadiansPerSecond() {
        return Units.rotationsToRadians(wristEncoder.getVelocity());
	}

	@Override 
    public void periodicallyCalled() {
        SmartDashboard.putNumber("Arm/Shoulder/Let Bus Voltage (V)", shoulderLeft.getBusVoltage());
        SmartDashboard.putNumber("Arm/Shoulder/Right Bus Voltage (V)", shoulderRight.getBusVoltage());
        SmartDashboard.putNumber("Arm/Wrist/Bus Voltage (V)", wrist.getBusVoltage());

        SmartDashboard.putNumber("Arm/Shoulder/Left Current (amps)", shoulderLeft.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Shoulder/Right Current (amps)", shoulderRight.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Wrist/Current (amps)", wrist.getOutputCurrent());

        SmartDashboard.putNumber("Arm/Shoulder/Raw Encoder Angle (rot)", shoulderEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Wrist/Raw Encoder Angle (rot)", wristEncoder.getPosition());

        SmartDashboard.putNumber("Arm/Shoulder/Left Duty Cycle", shoulderLeft.get());
        SmartDashboard.putNumber("Arm/Shoulder/Right Duty Cycle", shoulderRight.get());
        SmartDashboard.putNumber("Arm/Wrist/Duty Cycle", wrist.get());
	}

}
