package frc.robot.subsystems.drive;


import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public double heading = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {

    }
}