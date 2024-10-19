package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;


public interface ArmIO {
    

    @AutoLog
    public static class ArmIOInputs {
        public double motorCurrent;
        public double motorVoltage;
        public double motorPosition;
        public double motorVelocity;
        public double motorTemperature;
    }



    public default void setPosition(Measure<Angle> radians){
       
    }

    public default void updateInputs(ArmIOInputs inputs) {
    }
}