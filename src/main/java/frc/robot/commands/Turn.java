package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;

public class Turn extends Command {

    private final Drive drive;
    Rotation2d angleRead;
    double targetAngle;
    

    public Turn(Drive drive, double targetAngle) {
        this.drive = drive;
        this.targetAngle *= Math.PI / 180;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(targetAngle<0){
            drive.setVelocity(new ChassisSpeeds(0, 0, -1));
        }else{
            drive.setVelocity(new ChassisSpeeds(0, 0, 1));
        }
        this.angleRead = drive.getGyroRotation();

    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.angleRead.getDegrees()) >= targetAngle;
    }

    /* 
    @Override
    public void end (boolean interrupted) {
        drive.setVelocity(new ChassisSpeeds(0,0,0));
    }
    */

}