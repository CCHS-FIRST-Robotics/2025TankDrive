package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;

public class MoveForwardCommand extends Command {
    private final Drive drive;
    

    public MoveForwardCommand(Drive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        drive.setVelocity(new ChassisSpeeds(1, 0, 0));

    }

    @Override
    public boolean isFinished() {
        return drive.getLeftEncoderRotations().in(Rotations)>3 || drive.getRightEncoderRotations().in(Rotations)>3;
    }
}