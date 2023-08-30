package com.stuypulse.robot.subsystems.intake;

import static com.stuypulse.robot.constants.Motors.Intake.*;
import static com.stuypulse.robot.constants.Ports.Intake.*;
import static com.stuypulse.robot.constants.Settings.Intake.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

public class IntakeImpl extends Intake {

    private final CANSparkMax motor;

    private final BStream stalling;

    private State state;

    private boolean hasGamePiece;

    protected IntakeImpl() {
        motor = new CANSparkMax(MOTOR, MotorType.kBrushless);

        MOTOR_CONFIG.configure(motor);

        state = State.IDLE;

        hasGamePiece = false;

        stalling = BStream.create(this::isMomentarilyStalling)
            .filtered(new BDebounce.Rising(STALL_TIME));
    }

    @Override
    public void setState(State state) {
        if (this.state != state) {
            if (state != State.IDLE) {
                hasGamePiece = false;
            }
        }

        this.state = state;
    }

    @Override
    public State getState() {
        return state;
    }

    public void enableCoast() {
        motor.setIdleMode(IdleMode.kCoast);
    }

    public void enableBreak() {
        motor.setIdleMode(IdleMode.kBrake);
    }

    // stall detection

    private boolean isMomentarilyStalling() {
        double current = 35.0;
        if (state == State.INTAKING_CUBE) {
            current = CUBE_STALL_CURRENT.doubleValue();
        } else if (state == State.INTAKING_CONE) {
            current = CONE_STALL_CURRENT.doubleValue();
        }

        return motor.getOutputCurrent() > current;
    }

    private boolean isStalling() {
        return stalling.get();
    }

    @Override
    public boolean hasGamePiece() {
        return hasGamePiece;
    }

    @Override
    public void periodic() {
        motor.setVoltage(state.voltage);

        switch (state) {
            case INTAKING_CONE:
            case INTAKING_CUBE:
                if (stalling.get()) {
                    hasGamePiece = true;
                }

                if (hasGamePiece) {
                    setState(State.IDLE);
                }

                break;
            case IDLE:
            case OUTTAKING_CONE:
            case OUTTAKING_CUBE:
            case LOW_CONE_SPIT:
            case LOW_CUBE_SPIT:
        }

        SmartDashboard.putNumber("Intake/Motor Speed", motor.get());
        SmartDashboard.putNumber("Intake/Current", motor.getOutputCurrent());
        SmartDashboard.putBoolean("Intake/Is Stalling", isStalling());
        SmartDashboard.putString("Intake/State", state.toString());
        SmartDashboard.putNumber("Intake/Voltage", state.voltage);
    }
}