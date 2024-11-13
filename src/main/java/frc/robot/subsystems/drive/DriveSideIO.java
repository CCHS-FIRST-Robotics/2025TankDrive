package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface DriveSideIO {
    @AutoLog
    public static class DriveSideIOInputs {
        public Measure<Velocity<Angle>> currentSetpoint = RotationsPerSecond.of(0); // rotations per second
        
        public Measure<Current> motor1Current = Amps.of(0); // amps
        public Measure<Voltage> motor1Voltage = Volts.of(0); // volts
        public Measure<Angle> motor1Position = Rotations.of(0); // rotations
        public Measure<Velocity<Angle>> motor1Velocity = RotationsPerSecond.of(0); // rotations per second
        public Measure<Temperature> motor1Temperature = Celsius.of(0); // celcius
        public Measure<Angle> wheelPosition = Rotations.of(0); // rotations
        public Measure<Velocity<Angle>> wheelVelocity = RotationsPerSecond.of(0); // rotations per second
        public Measure<Distance> distanceTraveled = Meters.of(0); // meters
    }

    public default void setVoltage(Measure<Voltage> volts) {}

    public default void setVelocity(Measure<Velocity<Angle>> velocity) {}


    public default void setDriveBrakeMode(boolean enable) {}

    public default void updateInputs(DriveSideIOInputs inputs) {}
}