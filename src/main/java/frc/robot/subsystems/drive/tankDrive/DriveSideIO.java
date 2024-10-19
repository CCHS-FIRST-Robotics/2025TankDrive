package frc.robot.subsystems.drive.tankDrive;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.*;

public interface DriveSideIO {
    @AutoLog
    public static class DriveSideIOInputs {
        public double motor1Current;
        public double motor1Voltage;
        public double motor1Position;
        public double motor1Velocity;
        public double motor1Temperature;

        public double motor2Current;
        public double motor2Voltage;
        public double motor2Position;
        public double motor2Velocity;
        public double motor2Temperature;
    }

    public default void setVoltage(Measure<Voltage> volts) {}

    public default void setVelocity(Measure<Velocity<Angle>> velocity) {}

    public default void updateInputs(DriveSideIOInputs inputs) {}
}