package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.motors.GroupOfMotors;
import frc.robot.Constants;

public class ExponentialProfile extends Command{
    GroupOfMotors motors;
    int duration;
    int maxVoltage;
    int halfWavelength;
    
    double t = 0; 
    double change = 0;
    double totalVolts = 0;

    public ExponentialProfile(GroupOfMotors motors, int duration, int maxVoltage, int halfWavelength){
        addRequirements(motors);
        this.motors = motors;
        this.duration = duration;
        this.maxVoltage = maxVoltage;
        this.halfWavelength = halfWavelength;
    }
    
    @Override
    public void execute() {
        double changeChange = maxVoltage / (halfWavelength * (1 / Constants.PERIOD));
        if (((int) t / halfWavelength) % 2 == 0) { // increasing
            change += changeChange;
            totalVolts += change;
        } else { // decreasing
            change -= changeChange;
            totalVolts -= change;
        }

        /**
         * to normalize, you want the totalVolts at quarterWavelength
         * which is the average change(maxVoltage/2) times the number of 20ms in 10 seconds(500)
         * 6 * 500 = 3000
         */
        int maxTotalVolts = (maxVoltage/2) * (int)(halfWavelength / Constants.PERIOD);
        motors.setAllMotorVoltage(Volts.of((totalVolts / maxTotalVolts) * maxVoltage));

        t += 0.02; 
    }

    @Override
    public boolean isFinished(){
        return t > duration;
    }
}