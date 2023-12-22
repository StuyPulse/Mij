package com.stuypulse.robot.commands.auton;
import com.choreo.lib.*;
import com.stuypulse.robot.constants.Ports.Swerve;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static com.stuypulse.robot.constants.Settings.Swerve.Motion.*;

public class ChoreoMobility extends SequentialCommandGroup {
    ChoreoTrajectory trajectory = Choreo.getTrajectory("Mobility");

    public ChoreoMobility() {
        addCommands(
            Choreo.choreoSwerveCommand(
                trajectory,
                Odometry.getInstance()::getPose,
                new PIDController(XY.kP, XY.kI, XY.kD),
                new PIDController(XY.kP, XY.kI, XY.kD),
                new PIDController(THETA.kP, THETA.kI, THETA.kD),
                (ChassisSpeeds speeds) -> SwerveDrive.getInstance().setChassisSpeeds(speeds),
                false,
                SwerveDrive.getInstance()
            )
        );
    }    
}