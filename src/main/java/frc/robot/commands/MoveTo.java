package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.robot.subsystems.drive.Drive;

public class MoveTo extends Command {
    private final Drive drive;
    Measure<Distance> x;
    Measure<Velocity<Distance>> Mps;
    Measure<Distance> y;
    boolean finished;
    

    public MoveTo(
        Drive drive, 
        Measure<Distance> x, 
        Measure<Distance> y, 
        Measure<Velocity<Distance>> Mps)
     {

        this.drive = drive;
        this.x = x;
        this.Mps = Mps;
        this.y = y;
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
        return drive.goToPositsion(x,y,Mps);
     
    }
}