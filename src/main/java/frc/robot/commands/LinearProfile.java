package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.motors.GroupOfMotors;
import frc.robot.Constants;

public class LinearProfile extends Command{
    GroupOfMotors motors;
    int duration;
    int maxVoltage;
    int halfWavelength;
    
    double t = 0;
    double totalVolts = 0;

    public LinearProfile(GroupOfMotors motors, int duration, int maxVoltage, int halfWavelength){
        addRequirements(motors);
        this.motors = motors;
        this.duration = duration;
        this.maxVoltage = maxVoltage;
        this.halfWavelength = halfWavelength;
    }
    
    @Override
    public void execute() {
        double change = maxVoltage / (halfWavelength * (1 / Constants.PERIOD));
        
        if (((int) t / halfWavelength) % 2 == 0) { // increasing
            totalVolts += change;
        } else { // decreasing
            totalVolts -= change;
        }

        motors.setAllMotorVoltage(Volts.of(totalVolts));
        t += Constants.PERIOD; 
    }

    @Override
    public boolean isFinished(){
        return t > duration;
    }
}