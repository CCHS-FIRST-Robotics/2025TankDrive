package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;

public class MoveForwardCommand extends Command {
    private final Drive drive;

    int totalRotations = 10;
    double leftRotations;
    double rightRotations;
    

    public MoveForwardCommand(Drive drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        drive.setVelocity(new ChassisSpeeds(0.3, 0, 0));
        this.leftRotations = drive.getLeftEncoderRotations().in(Rotations);
        this.rightRotations = drive.getRightEncoderRotations().in(Rotations);

        System.out.println("Left Rotations: " + leftRotations);
        System.out.println("Right Rotations: " + rightRotations);

    }

    @Override
    public boolean isFinished() {
        return this.leftRotations > totalRotations && this.rightRotations > totalRotations;
    }

    /* 
    @Override
    public void end (boolean interrupted) {
        drive.setVelocity(new ChassisSpeeds(0,0,0));
    }
    */
}