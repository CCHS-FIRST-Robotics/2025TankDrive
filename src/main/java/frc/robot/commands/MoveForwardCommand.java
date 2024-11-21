package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;

public class MoveForwardCommand extends Command {
    private final Drive drive;

    double distanceTarget; 

    double leftStartingDistance; 
    double rightStartingDistance; 

    double leftDistance;
    double rightDistance;
    
    public MoveForwardCommand(Drive drive, double distanceTarget) {
        this.drive = drive;
        this.distanceTarget = distanceTarget;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        leftStartingDistance = drive.getLeftEncoderDistance();
        rightStartingDistance = drive.getRightEncoderDistance();
    }

    @Override
    public void execute() {
        drive.setVelocity(new ChassisSpeeds(0.5 * (distanceTarget > 0 ? 1 : -1), 0, 0));
        leftDistance = drive.getLeftEncoderDistance() - leftStartingDistance;
        rightDistance = drive.getRightEncoderDistance() - rightStartingDistance;
    } 

    @Override
    public boolean isFinished() {
        return Math.abs(leftDistance) > Math.abs(distanceTarget) || Math.abs(rightDistance) > Math.abs(distanceTarget);
    }
}