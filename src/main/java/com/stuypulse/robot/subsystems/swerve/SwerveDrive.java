package com.stuypulse.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.BackLeft;
import com.stuypulse.robot.constants.Settings.Swerve.BackRight;
import com.stuypulse.robot.constants.Settings.Swerve.FrontLeft;
import com.stuypulse.robot.constants.Settings.Swerve.FrontRight;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.module.SimModule;
import com.stuypulse.robot.subsystems.swerve.module.SwerveModule;
import com.stuypulse.robot.subsystems.swerve.module.SwerveModuleImpl;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

    public final static SwerveDrive instance;

    static {
        if (Robot.isReal()) {
            instance = new SwerveDrive(
                new SwerveModuleImpl(FrontRight.ID, FrontRight.MODULE_OFFSET, FrontRight.ABSOLUTE_OFFSET, Ports.Swerve.FrontRight.TURN, Ports.Swerve.FrontRight.DRIVE, Ports.Swerve.FrontRight.ENCODER),
                new SwerveModuleImpl(FrontLeft.ID, FrontLeft.MODULE_OFFSET, FrontLeft.ABSOLUTE_OFFSET, Ports.Swerve.FrontLeft.TURN, Ports.Swerve.FrontLeft.DRIVE, Ports.Swerve.FrontLeft.ENCODER),
                new SwerveModuleImpl(BackLeft.ID, BackLeft.MODULE_OFFSET, BackLeft.ABSOLUTE_OFFSET, Ports.Swerve.BackLeft.TURN, Ports.Swerve.BackLeft.DRIVE, Ports.Swerve.BackLeft.ENCODER),
                new SwerveModuleImpl(BackRight.ID, BackRight.MODULE_OFFSET, BackRight.ABSOLUTE_OFFSET, Ports.Swerve.BackRight.TURN, Ports.Swerve.BackRight.DRIVE, Ports.Swerve.BackRight.ENCODER)
            );
        }
        else {
            instance = new SwerveDrive(
                new SimModule(FrontRight.ID, FrontRight.MODULE_OFFSET),
                new SimModule(FrontLeft.ID, FrontLeft.MODULE_OFFSET),
                new SimModule(BackLeft.ID, BackLeft.MODULE_OFFSET),
                new SimModule(BackRight.ID, BackRight.MODULE_OFFSET)
            );
        }
    }

    public static SwerveDrive getInstance() {
        return instance;
    }
    
    private final SwerveModule[] modules;  
    private final SwerveDriveKinematics kinematics;
    private final AHRS gyro;
    private final FieldObject2d[] module2ds;
    
    public SwerveDrive(SwerveModule... modules) {    
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(getModuleOffsets());
        gyro = new AHRS(SPI.Port.kMXP);
        module2ds = new FieldObject2d[modules.length];
    }

    public void initModule2ds(Field2d field) {
        for (int i = 0; i < modules.length; i++) {
            module2ds[i] = field.getObject(modules[i].getID() + "-2d");
        }
    }
    /**
     * @return Returns translation of each module relative to the center of the robot.
     */
    public Translation2d[] getModuleOffsets() {
        var offsets = new Translation2d[modules.length];

        for (int i = 0; i < modules.length; i++) {
            offsets[i] = modules[i].getOffset();
        }

        return offsets;
    }

    /** 
     * @return Array with current state of each swerve module.
     */
    public SwerveModuleState[] getModuleStates() {
        var states = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }

        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        var positions = new SwerveModulePosition[modules.length];

        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }

        return positions;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());      
    }

    /**
     * Applies deadband to module state.
     * @param state
     * @return The original state if the speed is greater than the deadband, otherwise a state with zero speed
     */
    public SwerveModuleState filterModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) > Swerve.MODULE_VELOCITY_DEADBAND.get()) {
            return state;
        }

        return new SwerveModuleState(0, state.angle);
    }

    public void setModuleStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException(
                String.format("State count mismatch error: %d states does not equal %d modules", states.length, modules.length)
            );
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.MAX_MODULE_SPEED.get());

        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(filterModuleState(states[i]));
        }
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    public void drive(Vector2D translation, double rotation) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.y, -translation.x, -rotation, Odometry.getInstance().getRotation());

        // straight spinning drift fix
        Pose2d robotVelocity = new Pose2d(
            Settings.DT * speeds.vxMetersPerSecond,
            Settings.DT * speeds.vyMetersPerSecond,
            Rotation2d.fromRadians(Settings.DT * speeds.omegaRadiansPerSecond)
        );
        Twist2d twistVel = new Pose2d().log(robotVelocity);

        ChassisSpeeds updatedSpeeds = new ChassisSpeeds(
            twistVel.dx / Settings.DT,
            twistVel.dy / Settings.DT,
            twistVel.dtheta / Settings.DT
        );

        setChassisSpeeds(updatedSpeeds);
    }
    
    public void stop() {
        setChassisSpeeds(new ChassisSpeeds());
    }

    /* GYRO */
    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    public Rotation2d getGyroYaw() {
        return getGyroAngle();
    }

    // getGyroPitch() and getGyroRoll may need to be modified depending
    // on specifications of Mij drivetrain
    public Rotation2d getGyroPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    // may be robot specific depending on Mij drivetrain
    public double getForwardAccelerationGs() {
        return gyro.getWorldLinearAccelY(); // may be x instead of y
    }

    /* KINEMATICS */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    // TODO: figure out how this works
    public Rotation2d getBalanceAngle() {
        Rotation2d yaw = getGyroYaw(), pitch = getGyroPitch(), roll = getGyroRoll();
        
        double facingSlope = pitch.getTan() * yaw.getCos() + roll.getTan() * yaw.getSin();
        double maxSlope = Math.sqrt(Math.pow(roll.getTan(), 2) + Math.pow(pitch.getTan(), 2));

        SmartDashboard.putNumber("Swerve/Facing Slope", facingSlope);
        SmartDashboard.putNumber("Swerve/Max Slope", maxSlope);

        return Rotation2d.fromRadians(Math.signum(facingSlope) * Math.atan(maxSlope));
    }

    public void setXMode() {
        setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        });
    }

    public void setZeroMode() {
        setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0))
        });
    }
    
    @Override
    public void periodic() {
        Odometry odometry = Odometry.getInstance();
        Pose2d pose = odometry.getPose();
        Rotation2d angle = odometry.getRotation();

        for(int i = 0; i < module2ds.length; i++) {
            module2ds[i].setPose(new Pose2d(
                pose.getTranslation().plus(modules[i].getOffset().rotateBy(angle)),
                modules[i].getState().angle.plus(angle)
            ));
        }

        SmartDashboard.putNumber("Swerve/Balance Angle (deg)", getBalanceAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro Angle (deg)", getGyroAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro Pitch", getGyroPitch().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro Roll", getGyroRoll().getDegrees());

        SmartDashboard.putNumber("Swerve/Forward Acceleration (Gs)", getForwardAccelerationGs());
        SmartDashboard.putNumber("Swerve/X Acceleration (Gs)", gyro.getWorldLinearAccelX());
        SmartDashboard.putNumber("Swerve/Y Acceleration (Gs)", gyro.getWorldLinearAccelY());
        SmartDashboard.putNumber("Swerve/Z Acceleration (Gs)", gyro.getWorldLinearAccelZ());
    }

    @Override
    public void simulationPeriodic() {
        // Integrate omega in simulation and store in gyro
        var speeds = getKinematics().toChassisSpeeds(getModuleStates());

        gyro.setAngleAdjustment(gyro.getAngle() - Math.toDegrees(speeds.omegaRadiansPerSecond * Settings.DT));
    }

}
