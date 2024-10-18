package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import frc.robot.subsystems.drive.tankDrive.Drive;
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
            applyPreferences(leftY),
            0, 
            applyPreferences(rightX)
        );

        drive.setVelocity(speeds);
    }

    public double applyPreferences(double input){
        // add a deadzone
        if(Math.abs(input) < Constants.ANALOG_DEADZONE){
            return 0; 
        }

        return Math.pow(input, 2); // 2 is kinda a magic number
    }
}