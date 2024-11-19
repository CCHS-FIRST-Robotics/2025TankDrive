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
        this.targetAngle = targetAngle;
        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        drive.setVelocity(new ChassisSpeeds(0, 0, 2 * (targetAngle > 0 ? 1 : -1)));
        angleRead = drive.getGyroRotation();
    }

    @Override
    public boolean isFinished() {
        return targetAngle > 0 ? angleRead.getDegrees() > targetAngle : angleRead.getDegrees() < targetAngle;
    }
}