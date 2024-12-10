package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Degrees;

import static edu.wpi.first.units.Units.Meters;


import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import frc.robot.subsystems.drive.Drive;

public class TurnCommand extends Command {
    private final Drive drive;
    Measure<Angle> angle;
    boolean finished;
    Measure<Angle> target_Angle;
    Measure<Velocity<Angle>> Dps;
    

    public TurnCommand(
        Drive drive, 
        Measure<Angle> angle, 
        Measure<Velocity<Angle>> Dps) {

        this.drive = drive;
        this.angle = angle;
        this.finished = false;
        this.target_Angle = Degrees.of(0);
        this.Dps = Dps;

    }

    @Override
    public void initialize() {
        target_Angle = Degrees.of(drive.getHeading() + angle.in(Degrees)); 
    }

    @Override
    public void execute() {
        finished = drive.goForward(target_Angle, 0, 0, Dps );

    }

    @Override
    public boolean isFinished() {

        return finished;
     
    }
}