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

    public DriveWithJoysticks(
        Drive drive,
        Supplier<Double> leftYSupplier,
        Supplier<Double> leftXSupplier
    ){
        addRequirements(drive);
        this.drive = drive;
        this.leftYSupplier = leftYSupplier;
        this.leftXSupplier = leftXSupplier;
    }

    @Override
    public void execute() {
        double leftY = leftYSupplier.get();
        double leftX = leftXSupplier.get();

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            convertToVelocity(leftY),
            0, 
            convertToVelocity(leftX) * -1 // chassisspeeds considers rotating clockwise as positive
        );

        drive.setVelocity(chassisSpeeds);
    }

    public double convertToVelocity(double input){
        if(Math.abs(input) < Constants.ANALOG_DEADZONE){
            return 0; 
        }
        return Math.signum(input) * Math.pow(Math.abs(input), Constants.JOYSTICK_EXPONENT) * Constants.MAX_SPEED;
    }
}