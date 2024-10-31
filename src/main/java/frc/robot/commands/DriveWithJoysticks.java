package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;



import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveWithJoysticks extends Command {
    private final Drive drive;
    private final Supplier<Double> leftYSupplier;
    private final Supplier<Double> leftXSupplier;
    double Kp;
    double Ki;
    double Kd;
    boolean piding;

    ChassisSpeeds speeds;
    private final PIDController pidController;
    Rotation2d targetHeading;


    public DriveWithJoysticks(
        Drive drive,
        Supplier<Double> leftYSupplier,
        Supplier<Double> leftXSupplier
    
    ) {
        addRequirements(drive);
        this.Kp = .0000001;
        this.Ki = 0.1;
        this.Kd = 0.0;
        this.drive = drive;
        this.leftYSupplier = leftYSupplier;
        this.leftXSupplier = leftXSupplier;
        this.piding = false;
        speeds = new ChassisSpeeds(
        0.0,
        0.0,
        0.0);
        this.targetHeading = new Rotation2d();
       

        
        this.pidController = new PIDController(Kp, Ki, Kd);
    }

    @Override
public void execute() {
    double leftY = leftYSupplier.get(); 
    double leftX = leftXSupplier.get(); 
    Rotation2d currentHeading = drive.getHeading();
    if( applyPreferences(leftX) == 0 && applyPreferences(leftY) != 0 &&  !this.piding){
        setTargetHeading(currentHeading.getRadians());
        this.piding = true;
    }

    if(applyPreferences(leftX) != 0){
        piding = false;
    }

  
    double headingError = this.targetHeading.getRadians() - currentHeading.getRadians();
    double pidOutput = pidController.calculate(headingError);

    if(this.piding){
    speeds = new ChassisSpeeds(
        applyPreferences(leftY),
        0, 
        -applyPreferences(leftX) * 2 + pidOutput 
    );
    }

    else{
    speeds = new ChassisSpeeds(
        applyPreferences(leftY),
        0, 
        -applyPreferences(leftX) * 2 
    );
    }

    drive.setVelocity(speeds);
    Logger.recordOutput("target heading", this.targetHeading.getRadians());
    Logger.recordOutput("Raw X output", -applyPreferences(leftX) * 2);
    Logger.recordOutput("mathed X output", -applyPreferences(leftX) * 2 + pidOutput );
    Logger.recordOutput("pid output", pidOutput );
    Logger.recordOutput("pidding", this.piding );
  
}


    public double applyPreferences(double input) {
        if (Math.abs(input) < Constants.ANALOG_DEADZONE) {
            return 0; 
        }
        return Math.signum(input) * Math.pow(Math.abs(input), Constants.JOYSTICK_EXPONENT) * Constants.MAX_SPEED; // 4 meters per second
    }

    public void setTargetHeading(double headingRadians) {
        this.targetHeading = Rotation2d.fromRadians(headingRadians);
    }
    
}
