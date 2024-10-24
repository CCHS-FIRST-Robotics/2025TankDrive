package frc.robot.subsystems.drive;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface DriveSideIO {
    @AutoLog
    public static class DriveSideIOInputs {
        public Measure<Velocity<Angle>> currentSetpoint; // rotations per second
        
        public Measure<Current> motor1Current; // amps
        public Measure<Voltage> motor1Voltage; // volts
        public Measure<Angle> motor1Position; // rotations
        public Measure<Velocity<Angle>> motor1Velocity; // rotations per second
        public Measure<Temperature> motor1Temperature; // celcius
    }

    public default void setVoltage(Measure<Voltage> volts) {}

    public default void setVelocity(Measure<Velocity<Angle>> velocity) {}

    public default void updateInputs(DriveSideIOInputs inputs) {}
}