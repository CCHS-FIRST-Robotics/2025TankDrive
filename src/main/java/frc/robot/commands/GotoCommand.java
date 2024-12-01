package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Degrees;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import frc.robot.subsystems.drive.Drive;

public class GotoCommand extends Command {
    private final Drive drive;
    Measure<Distance> x;
    Measure<Velocity<Distance>> Mps;
    Measure<Distance> y;
    boolean finished;
    Measure<Distance> target_Meters;
    Measure<Angle> target_Angle;
    Measure<Velocity<Angle>> Dps;
    

    public GotoCommand(
        Drive drive, 
        Measure<Distance> x, 
        Measure<Distance> y,
        Measure<Velocity<Distance>> Mps,
        Measure<Velocity<Angle>> Dps) {

        this.drive = drive;
        this.target_Angle = Degrees.of(0);
        this.Mps = Mps; 
        this.finished = false;
        this.target_Meters = Meters.of(0);
        this.target_Angle = Degrees.of(0);
        this.Dps = Dps;

    }

    @Override
    public void initialize() {
        target_Meters = Meters.of(((drive.getlefttravelled() + drive.getrighttravelled()) / 2) + (Math.sqrt(Math.pow(x.in(Meters),2) + Math.pow(y.in(Meters),2))));
        target_Angle = Degrees.of(drive.getHeading() + (Math.atan2(y.in(Meters), x.in(Meters)) * ( 180/Math.PI)));
    }

    @Override
    public void execute() {
        finished = drive.goForward(target_Angle, target_Meters, Mps, Dps );

    }

    @Override
    public boolean isFinished() {

        return finished;
     
    }
}