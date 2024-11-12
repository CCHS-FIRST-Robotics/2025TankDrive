package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.annotation.Target;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class MoveForwardCommand extends Command {
    private final Drive drive;
    Measure<Angle> angle;
    Measure<Velocity<Distance>> Mps;
    Measure<Distance> distance;
    boolean finished;
    Measure<Angle> rotations;
    Measure<Angle> target_Angle;
    

    public MoveForwardCommand(
        Drive drive, 
        Measure<Angle> angle, 
        Measure<Velocity<Distance>> Mps, 
        Measure<Distance> distance) {

        this.drive = drive;
        this.angle = angle;
        this.Mps = Mps;
        this.distance = distance;
        this.finished = false;
        this.rotations = Rotations.of(0);
        this.target_Angle = Degrees.of(0);

    }

    @Override
    public void initialize() {
        drive.setDriveBrakeMode(true);
        rotations = Rotations.of(-((drive.getLeftRotations().in(Rotations) + drive.getRightRotations().in(Rotations)) / 2) + (distance.in(Meters) / Constants.WHEEL_CIRCUMFERENCE)) ;
        target_Angle = Degrees.of(drive.getHeading()); 
    }

    @Override
    public void execute() {
        
        finished = drive.goForward(target_Angle, Mps, rotations);

    }

    @Override
    public boolean isFinished() {
        
        return finished;
     
    }
}