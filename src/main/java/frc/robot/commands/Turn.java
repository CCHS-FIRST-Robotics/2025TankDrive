package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;

public class Turn extends Command {
    private final Drive drive;

    double wheelRotations = 5;

    Rotation2d angleRead;
    

    public Turn(Drive drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        drive.setVelocity(new ChassisSpeeds(0, 0, 0.5));
        this.angleRead = drive.getGyroRotation();

    }

    @Override
    public boolean isFinished() {
        return this.angleRead.getRadians() > Math.PI/2;
    }

    /* 
    @Override
    public void end (boolean interrupted) {
        drive.setVelocity(new ChassisSpeeds(0,0,0));
    }
    */

}