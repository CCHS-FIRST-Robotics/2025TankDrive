package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;

public class MoveForwardCommand extends Command {
    private final Drive drive;

    double wheelRotations = 5;

    double leftWheelRotations;
    double rightWheelRotations;
    

    public MoveForwardCommand(Drive drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        drive.setVelocity(new ChassisSpeeds(0.5, 0, 0));
        this.leftWheelRotations = drive.getLeftEncoderWheelRotations();
        this.rightWheelRotations = drive.getRightEncoderWheelRotations();

        System.out.println("Left Rotations: " + leftWheelRotations);
        System.out.println("Right Rotations: " + rightWheelRotations);

    }

    @Override
    public boolean isFinished() {
        return this.leftWheelRotations > wheelRotations && this.rightWheelRotations > wheelRotations;
    }

    /* 
    @Override
    public void end (boolean interrupted) {
        drive.setVelocity(new ChassisSpeeds(0,0,0));
    }
    */

}