package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import frc.robot.subsystems.motors.GroupOfMotors;
import frc.robot.Constants;

public class VoltageWithJoysticks extends Command{
    GroupOfMotors motors;
    Supplier<Double> leftXSupplier;

    public VoltageWithJoysticks(
        GroupOfMotors motors,
        Supplier<Double> leftXSupplier
    ){
        addRequirements(motors);
        this.motors = motors;
        this.leftXSupplier = leftXSupplier;
    }
    
    @Override
    public void execute() {
        double leftX = leftXSupplier.get();
        
        motors.setAllMotorVoltage(Volts.of(applyPreferences(leftX)));
    }

    public double applyPreferences(double input){
        if(Math.abs(input) < Constants.ANALOG_DEADZONE){
            return 0; 
        }
        return Math.pow(input, 2) * Math.signum(input) * 8;
    }
}