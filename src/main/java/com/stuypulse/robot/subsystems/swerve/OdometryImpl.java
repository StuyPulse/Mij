package com.stuypulse.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OdometryImpl extends Odometry {
    public final SwerveDriveOdometry odometry;
    // public final SwerveDrivePoseEstimator poseEstimator;

    public final Field2d field;
    public final FieldObject2d odometryPose2d;
    // public final FieldObject2d poseEstimatorPose2d;

    // Vector<N3> AUTO_STDDEVS = VecBuilder.fill(0.3, 0.3, Math.toRadians(30));
    // Vector<N3> TELEOP_STDDEVS = VecBuilder.fill(0.3 - Units.inchesToMeters(5.0), 0.3 - Units.inchesToMeters(5.0), Units.degreesToRadians(30));

    protected OdometryImpl() {
        SwerveDrive swerve = SwerveDrive.getInstance();
        Pose2d startingPose = new Pose2d();

        odometry = new SwerveDriveOdometry(
            swerve.getKinematics(),
            swerve.getGyroAngle(), 
            swerve.getModulePositions(),
            startingPose
        );
        
        // Use if we need CV
        // poseEstimator = new SwerveDrivePoseEstimator(
        //     swerve.getKinematics(),
        //     swerve.getGyroAngle(), 
        //     swerve.getModulePositions(),
        //     startingPose,
        //     VecBuilder.fill(0.1, 0.1, 0.1),
        //     AUTO_STDDEVS
        //  );
        field = new Field2d();

        odometryPose2d = field.getObject("Odometry Pose2d");
        // poseEstimatorPose2d = field.getObject("Pose Estimator Pose2d");
        odometryPose2d.setPose(startingPose);

        swerve.initModule2ds(field);
        SmartDashboard.putData("Field", field);
    }

    public Field2d getField() {
        return field;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    
    @Override
    public void periodic() {
        SwerveDrive swerve = SwerveDrive.getInstance();
        odometry.update(swerve.getGyroAngle(), swerve.getModulePositions());
        odometryPose2d.setPose(getPose());

        SmartDashboard.putNumber("Odometry/Odometry Pose X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry/Odometry Pose Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry/Odometry Rotation", odometry.getPoseMeters().getRotation().getDegrees()); 
    }
}
