package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface DriveSideIO {
    @AutoLog
    public static class DriveSideIOInputs {
        public Measure<Velocity<Angle>> currentSetpoint = RotationsPerSecond.of(0); // rotations per second
        
        public Measure<Current> motorCurrent = Amps.of(0); // amps
        public Measure<Voltage> motorVoltage = Volts.of(0); // volts
        public Measure<Angle> motorPosition = Rotations.of(0); // rotations
        public Measure<Velocity<Angle>> motorVelocity = RotationsPerSecond.of(0); // rotations per second
    }

    public default void setVoltage(Measure<Voltage> volts) {}

    public default void setVelocity(Measure<Velocity<Angle>> velocity) {}

    public default void updateInputs(DriveSideIOInputs inputs) {}
}