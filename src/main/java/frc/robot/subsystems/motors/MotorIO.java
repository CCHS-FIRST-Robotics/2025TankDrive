package frc.robot.subsystems.motors;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.*;

public interface MotorIO {
    @AutoLog
    public static class MotorIOInputs {
        public double motorCurrent;
        public double motorVoltage;
        public double motorPosition;
        public double motorVelocity;
        public double motorTemperature;
    }

    public default void setVoltage(Measure<Voltage> volts) {}

    public default void setPosition(Measure<Angle> position){}

    public default void updateInputs(MotorIOInputs inputs) {}
}