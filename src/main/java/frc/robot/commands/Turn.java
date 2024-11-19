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
        drive.setVelocity(new ChassisSpeeds(0, 0, (targetAngle > 0 ? -1 : 1)));

        this.angleRead = drive.getGyroRotation();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.angleRead.getDegrees()) > Math.abs(targetAngle);
    }

    /* 
    @Override
    public void end (boolean interrupted) {
        drive.setVelocity(new ChassisSpeeds(0,0,0));
    }
    */

}