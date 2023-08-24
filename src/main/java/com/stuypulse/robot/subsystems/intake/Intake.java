package com.stuypulse.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {
    
    //singleton
    private static final Intake instance;

    static {
        instance = new IntakeImpl();
    }

    public static Intake getInstance() {
        return instance;
    }

    // state storage
    public enum State {
        IDLE(0.0), 
        INTAKING_CUBE(-12.0), 
        OUTTAKING_CUBE(12.0),
        
        INTAKING_CONE(12.0),
        OUTTAKING_CONE(-12.0),
        
        LOW_CUBE_SPIT(8.0),
        LOW_CONE_SPIT(-8.0);

        public double voltage;
        State (double voltage) {
            this.voltage = voltage;
        }
    }

    public abstract void setState(State state);
    public abstract State getState();

    public abstract boolean hasGamePiece();
}
