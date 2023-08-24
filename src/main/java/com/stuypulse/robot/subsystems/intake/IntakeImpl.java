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

    private CANSparkMax motor;

    private BStream stalling;

    private boolean acquiring;

    public State mState = State.IDLE;

    private boolean hasGamePiece = false;

    protected IntakeImpl() {
        motor = new CANSparkMax(MOTOR, MotorType.kBrushless);

        MOTOR_CONFIG.configure(motor);

        stalling = BStream.create(this::isMomentarilyStalling)
            .filtered(new BDebounce.Rising(STALL_TIME));
    }

    public void setState(State state) {
        if (mState != state) {
            if (state != State.IDLE) {
                hasGamePiece = false;
            }
        }
    }

    public State getState() {
        return mState;
    }

    public void enableCoast() {
        motor.setIdleMode(IdleMode.kCoast);
    }

    public void enableBreak() {
        motor.setIdleMode(IdleMode.kBrake);
    }

    // stall detection

    private boolean isMomentarilyStalling() {
        return motor.getOutputCurrent() > STALL_CURRENT.doubleValue();
    }

    private boolean isStalling() {
        return stalling.get();
    }

    @Override
    public boolean hasGamePiece() {
        return isStalling();
    }

    @Override
    public void periodic() {

        if (acquiring && hasGamePiece()) {
            setState(State.IDLE);
        }

        //TO DO
        switch (mState) {
            case IDLE:
            case INTAKING_CONE:
            case INTAKING_CUBE:
            case OUTTAKING_CONE:
            case OUTTAKING_CUBE:
            case LOW_CONE_SPIT:
            case LOW_CUBE_SPIT:
        }

        SmartDashboard.putNumber("Intake Motor Speed", motor.get());
        SmartDashboard.putNumber("Intake Current", motor.getOutputCurrent());
        SmartDashboard.putBoolean("Intake/Is Stalling", isStalling());
    }
}