package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;

public class MoveForwardCommand extends Command {
    private final Drive drive;

    double wheelRotationsTarget;

    double leftWheelRotations;
    double rightWheelRotations;

    double distanceTarget; 

    double leftDistance;
    double rightDistance;
    
    public MoveForwardCommand(Drive drive, double wheelRotations) {
        this.drive = drive;
        this.wheelRotationsTarget = wheelRotations;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        drive.setVelocity(new ChassisSpeeds(0.8 * (wheelRotationsTarget > 0 ? 1 : -1), 0, 0));
        leftWheelRotations = drive.getLeftEncoderWheelRotations();
        rightWheelRotations = drive.getRightEncoderWheelRotations();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(leftWheelRotations) > Math.abs(wheelRotationsTarget) && Math.abs(rightWheelRotations) > Math.abs(wheelRotationsTarget);
    }

    /* 
    @Override
    public void end (boolean interrupted) {
        drive.setVelocity(new ChassisSpeeds(0,0,0));
    }
    */

}