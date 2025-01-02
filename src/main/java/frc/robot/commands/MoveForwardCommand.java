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

    int halfWavelen;
    int duration;
    double t = 0;
    double totalSpeed = 0;
    double maxVelocity = 3; // 3 m/s
    
    public MoveForwardCommand(Drive drive, double distanceTarget, int halfWavelen, int duration) {
        addRequirements(drive);

        this.drive = drive;
        this.distanceTarget = distanceTarget;

        this.halfWavelen = halfWavelen;
        this.duration = duration;
    }

    @Override
    public void initialize() {
        leftStartingDistance = drive.getLeftEncoderDistance();
        rightStartingDistance = drive.getRightEncoderDistance();
    }

    @Override
    public void execute() {
        double change = maxVelocity / (halfWavelen * (1 / 0.02));

        if (((int) t / halfWavelen) % 2 == 0) { //accelerating
            change += change;
            totalSpeed += change;
        } else { //decelerating
            change -= change;
            totalSpeed -= change;
        }

        drive.setVelocity(new ChassisSpeeds(totalSpeed * (distanceTarget > 0 ? 1 : -1), 0, 0));
        leftDistance = drive.getLeftEncoderDistance() - leftStartingDistance;
        rightDistance = drive.getRightEncoderDistance() - rightStartingDistance;
    } 

    @Override
    public boolean isFinished() {
        return Math.abs(leftDistance) > Math.abs(distanceTarget) || Math.abs(rightDistance) > Math.abs(distanceTarget);
    }
}