package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;

public class MoveForwardCommand extends Command {
    private final Drive drive;

    int totalRotations = 5;
    

    public MoveForwardCommand(Drive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        drive.setVelocity(new ChassisSpeeds(2, 0, 0));

    }

    @Override
    public boolean isFinished() {
        System.out.println(drive.getLeftEncoderRotations().in(Rotations));
        System.out.println(-drive.getRightEncoderRotations().in(Rotations));
     
        return drive.getLeftEncoderRotations().in(Rotations) > totalRotations || 
            drive.getRightEncoderRotations().in(Rotations) > totalRotations;
    }
}