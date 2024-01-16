/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import java.util.Optional;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.swerve.SysId;
import com.stuypulse.robot.subsystems.test.TestDrive;
import com.stuypulse.robot.subsystems.test.TestDriveSim;
import com.stuypulse.robot.util.BootlegXbox;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new BootlegXbox(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new BootlegXbox(Ports.Gamepad.OPERATOR);

    
    // Subsystem
    // public final TestDrive testDrive = new TestDriveSim();
    public final SysId swerve = new SysId();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();
    private static Optional<Alliance> cachedAlliance;

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());
        // autonChooser.addOption("8 Feet", new EightFootAuton());
        // autonChooser.setDefaultOption("Mobility", new Mobility());
        // autonChooser.addOption("Mobility Bump", new MobilityBump());
        // autonChooser.addOption("Drive And Turn Bump", new DriveAndTurnBump());

        // autonChooser.addOption("Test - Quasistatic Forward", testDrive.quasistaticForward());
        // autonChooser.addOption("Test - Quasistatic Reverse", testDrive.quasistaticReverse());
        // autonChooser.addOption("Test - Dynamic Forward", testDrive.dynamicForward());
        // autonChooser.addOption("Test - Dynamic Reverse", testDrive.dynamicReverse());

        autonChooser.addOption("Swerve SysId - Quasistatic Forward", swerve.quasistaticForward());
        autonChooser.addOption("Swerve SysId - Quasistatic Reverse", swerve.quasistaticReverse());
        autonChooser.addOption("Swerve SysId - Dynamic Forward", swerve.dynamicForward());
        autonChooser.addOption("Swerve SysId - Dynamic Reverse", swerve.dynamicReverse());
        
        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public static void setCachedAlliance(Optional<Alliance> alliance) {
        cachedAlliance = DriverStation.getAlliance();
    }

    public static Optional<Alliance> getCachedAlliance() {
        return cachedAlliance;
    }
}
