/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import java.util.Optional;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.DriveAndTurnBump;
import com.stuypulse.robot.commands.auton.EightFootAuton;
import com.stuypulse.robot.commands.auton.Mobility;
import com.stuypulse.robot.commands.auton.MobilityBump;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetHeading;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.swerve.TestDrive;
import com.stuypulse.robot.subsystems.swerve.TestDriveSim;
import com.stuypulse.robot.util.BootlegXbox;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new BootlegXbox(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new BootlegXbox(Ports.Gamepad.OPERATOR);

    public final SwerveDrive swerve = SwerveDrive.getInstance();
    public final Odometry odometry = Odometry.getInstance();
    
    // Subsystem
    public final TestDrive testDrive = new TestDriveSim();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();
    private static Alliance cachedAlliance;

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
        // Swerve
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        driver.getDPadUp().onTrue(new SwerveDriveResetHeading(Rotation2d.fromDegrees(180)));
        driver.getDPadDown().onTrue(new SwerveDriveResetHeading(Rotation2d.fromDegrees(0)));
        driver.getDPadLeft().onTrue(new SwerveDriveResetHeading(Rotation2d.fromDegrees(270)));
        driver.getDPadRight().onTrue(new SwerveDriveResetHeading(Rotation2d.fromDegrees(90)));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.addOption("Do Nothing", new DoNothingAuton());
        // autonChooser.addOption("8 Feet", new EightFootAuton());
        // autonChooser.setDefaultOption("Mobility", new Mobility());
        // autonChooser.addOption("Mobility Bump", new MobilityBump());
        // autonChooser.addOption("Drive And Turn Bump", new DriveAndTurnBump());

        // autonChooser.addOption("Test - Quasistatic Forward", testDrive.quasistaticForward());
        // autonChooser.addOption("Test - Quasistatic Reverse", testDrive.quasistaticReverse());
        // autonChooser.addOption("Test - Dynamic Forward", testDrive.dynamicForward());
        // autonChooser.addOption("Test - Dynamic Reverse", testDrive.dynamicReverse());

        autonChooser.addOption("SysId - Quasistatic Forward", swerve.sysId.quasistaticForward());
        autonChooser.addOption("SysId - Quasistatic Reverse", swerve.sysId.quasistaticReverse());
        autonChooser.addOption("SysId - Dynamic Forward", swerve.sysId.dynamicForward());
        autonChooser.addOption("SysId - Dynamic Reverse", swerve.sysId.dynamicReverse());
        
        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public static void setCachedAlliance(Optional<Alliance> alliance) {
        cachedAlliance = DriverStation.getAlliance().get();
    }

    public static Alliance getCachedAlliance() {
        return cachedAlliance;
    }
}
