package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class RobotConfig {
    private final DifferentialDriveKinematics kinematics;
    private final double maxSpeed;
    private final double maxAcceleration;

    public RobotConfig(DifferentialDriveKinematics kinematics, double maxSpeed, double maxAcceleration) {
        this.kinematics = kinematics;
        this.maxSpeed = maxSpeed;
        this.maxAcceleration = maxAcceleration;
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }
}

