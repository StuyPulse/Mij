package com.stuypulse.robot.commands.swerve;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.vectors.VStream;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveDrive extends CommandBase {
       
    private SwerveDrive swerve;
    
    private VStream speed; 
    private IStream turn;

    private final Gamepad driver;

    private Optional<Rotation2d> holdAngle;

    public SwerveDriveDrive(Gamepad driver) {
        this.driver = driver;
        swerve = SwerveDrive.getInstance();
    
        speed = VStream.create(driver::getLeftStick);
        turn = IStream.create(driver::getRightX);       
        
        holdAngle = Optional.empty();
        
        addRequirements(swerve);

        
    }

    @Override
    public void initialize() {
        holdAngle = Optional.empty();
    }

    private boolean isWithinTurnDeadband() {
        return Math.abs(turn.get()) < Swerve.MAX_MODULE_TURN.get();
    }

    private boolean isWithinDriveDeadband() {
        return Math.abs(speed.get().magnitude()) < Swerve.MAX_MODULE_SPEED.get();
    }
   
   @Override
    public void execute() {
        swerve.drive(speed.get(), turn.get());


        if(isWithinTurnDeadband()){
            if(holdAngle.isEmpty()) {
                holdAngle = Optional.of(swerve.getGyroAngle());
            }

        }
        /*if (isWithinDriveDeadband()) {
            if () {

            }
            if () {

            }
        }

        else {

        }*/
    }
}