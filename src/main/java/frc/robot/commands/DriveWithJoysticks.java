package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Constants;

public class DriveWithJoysticks extends Command{
    Drive drive;
    Supplier<Double> leftYSupplier;
    Supplier<Double> leftXSupplier;
    
    public DriveWithJoysticks(Drive drive, Supplier<Double> leftYSupplier, Supplier<Double> leftXSupplier) {
        addRequirements(drive);
        this.drive = drive;
        this.leftYSupplier = leftYSupplier;
        this.leftXSupplier = leftXSupplier;
    }
    
    @Override
    public void execute() {
        double leftY = leftYSupplier.get();
        double leftX = leftXSupplier.get();

        //x translation, y translation, rotation
        ChassisSpeeds speeds = new ChassisSpeeds(applyPreferences(leftY), 0, -applyPreferences(leftX) * 2); // chassisspeeds makes clockwise positive

        drive.setVelocity(speeds);
    }

    public double applyPreferences(double input){
        double scaledExponential = Math.pow(Math.abs(input), Constants.JOYSTICK_EXPONENT);

        if (Math.abs(input) < Constants.ANALOG_DEADZONE){
            return 0; 
        }
        return Math.signum(input) * scaledExponential * Constants.MAX_SPEED; // 4 m/s
    }
}