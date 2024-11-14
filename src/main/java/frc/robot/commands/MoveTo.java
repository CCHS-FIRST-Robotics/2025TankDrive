package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;


import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class MoveTo extends Command {
    private final Drive drive;
    Measure<Distance> x;
    Measure<Velocity<Distance>> Mps;
    Measure<Distance> y;
    boolean finished;
    Measure<Angle> target_Rotations;
    Measure<Angle> target_Angle;
    

    public MoveTo(
        Drive drive, 
        Measure<Distance> x, 
        Measure<Distance> y, 
        Measure<Velocity<Distance>> Mps) {

        this.drive = drive;
        this.x = x;
        this.Mps = Mps;
        this.y = y;
        this.finished = false;
        this.target_Rotations = Rotations.of(0);
        this.target_Angle = Degrees.of(0);

    }

    @Override
    public void initialize() {
        drive.setDriveBrakeMode(true);

        target_Rotations = Rotations.of((Math.sqrt(Math.pow(x.in(Meters), 2) + Math.pow(y.in(Meters), 2))) / Constants.WHEEL_CIRCUMFERENCE.in(Meters));
        target_Angle = Degrees.of(Math.atan2(y.in(Meters), x.in(Meters)));
    }

    @Override
    public void execute() {
        
        finished = drive.goForward(target_Angle, Mps, target_Rotations);

    }

    @Override
    public boolean isFinished() {

        return finished;
     
    }
}