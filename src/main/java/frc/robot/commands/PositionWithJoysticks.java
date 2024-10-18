package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import frc.robot.subsystems.motors.GroupOfMotors;

public class PositionWithJoysticks extends Command{
    GroupOfMotors motors;
    Supplier<Double> leftXSupplier;

    public PositionWithJoysticks(
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
        
        motors.setAllMotorPosition(Rotations.of(leftX));
    }
}