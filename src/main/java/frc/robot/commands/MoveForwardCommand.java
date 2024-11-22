package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Degrees;

import static edu.wpi.first.units.Units.Meters;
;


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
    Measure<Distance> target_Meters;
    Measure<Angle> target_Angle;
    Measure<Velocity<Angle>> Dps;
    

    public MoveForwardCommand(
        Drive drive, 
        Measure<Angle> angle, 
        Measure<Distance> distance,
        Measure<Velocity<Distance>> Mps,
        Measure<Velocity<Angle>> Dps) {

        this.drive = drive;
        this.angle = angle;
        this.Mps = Mps; 
        this.distance = distance; 
        this.finished = false;
        this.target_Meters = Meters.of(0);
        this.target_Angle = Degrees.of(0);
        this.Dps = Dps;

    }

    @Override
    public void initialize() {
        target_Meters = Meters.of(((drive.getlefttravelled() + drive.getrighttravelled()) / 2) + (distance.in(Meters) ));
        target_Angle = Degrees.of(drive.getHeading() + angle.in(Degrees)); 
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