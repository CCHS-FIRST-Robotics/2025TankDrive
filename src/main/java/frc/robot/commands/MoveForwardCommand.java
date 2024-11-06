package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.robot.subsystems.drive.Drive;

public class MoveForwardCommand extends Command {
    private final Drive drive;
    Measure<Angle> angle;
    Measure<Velocity<Distance>> Mps;
    Measure<Distance> distance;
    boolean finished;
    

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

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return drive.goForward(angle, Mps, distance);
     
    }
}