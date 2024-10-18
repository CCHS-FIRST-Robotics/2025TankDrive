package frc.robot.subsystems.pneumatics;

import org.littletonrobotics.junction.AutoLog;

public interface SolenoidIO {
    @AutoLog
    public static class SolenoidIOInputs {
        public boolean on;
    }

    public default void set(boolean on) {
    }

    public default void toggle() {
    }

    public default void updateInputs(SolenoidIOInputs inputs) {
    }
}