package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Constants;

public class DriveWithJoysticks extends Command{
    Drive drive;
    Supplier<Double> leftYSupplier;
    Supplier<Double> rightXSupplier;
    
    public DriveWithJoysticks(
        Drive drive,
        Supplier<Double> leftYSupplier,
        Supplier<Double> rightXSupplier
    ){
        addRequirements(drive);
        this.drive = drive;
        this.leftYSupplier = leftYSupplier;
        this.rightXSupplier = rightXSupplier;
    }
    
    @Override
    public void execute() {
        double leftY = leftYSupplier.get();
        double rightX = rightXSupplier.get();

        ChassisSpeeds speeds = new ChassisSpeeds(
            -applyPreferences(leftY), // xboxcontroller is so quirky
            0, 
            -applyPreferences(rightX) * 2 // chassisspeeds considers rotating clockwise as positive
        );

        drive.setVelocity(speeds);
    }

    public double applyPreferences(double input){
        if(Math.abs(input) < Constants.ANALOG_DEADZONE){
            return 0; 
        }
        return Math.signum(input) * Math.pow(input, 2) * Constants.MAX_SPEED; // 4 meters per second
    }
}