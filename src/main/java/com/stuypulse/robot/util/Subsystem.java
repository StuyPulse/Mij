package com.stuypulse.robot.util;

import java.util.ArrayList;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Subsystem extends SubsystemBase {

    private String name;

    private ArrayList<Runnable> periodicFunctions;

    public Subsystem(String name) {
        this.name = name;

        periodicFunctions = new ArrayList<Runnable>();
    }

    /** SMARTDASHBOARD WRAPPERS **/

    public final void setName(String name) {
        this.name = name;
    }

    public final void putData(String key, Sendable data) {
        SmartDashboard.putData(name + "/" + key, data);   
    }

    public final void putBoolean(String key, boolean value) {
        SmartDashboard.putBoolean(name + "/" + key, value);   
    }

    public final void putNumber(String key, double value) {
        SmartDashboard.putNumber(name + "/" + key, value);   
    }

    public final void putString(String key, String value) {
        SmartDashboard.putString(name + "/" + key, value);   
    }

    public final void putBooleanArray(String key, boolean[] value) {
        SmartDashboard.putBooleanArray(name + "/" + key, value);   
    }

    public final void putNumberArray(String key, double[] value) {
        SmartDashboard.putNumberArray(name + "/" + key, value);   
    }

    public final void putStringArray(String key, String[] value) {
        SmartDashboard.putStringArray(name + "/" + key, value);   
    }

    public final void putRaw(String key, byte[] value) {
        SmartDashboard.putRaw(name + "/" + key, value);
    }

    /** PERIODIC **/

    public final void registerPeriodicFunc(Runnable periodicFunc) {
        periodicFunctions.add(periodicFunc);
    }
    
    @Override
    public final void periodic() {
        for (Runnable func : periodicFunctions) {
            func.run();
        }
    }

}
