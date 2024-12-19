package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class RobotConfig {
    private final DifferentialDriveKinematics width;
    private final double length;
    private final double maxSpeed;
    private final double maxAcceleration;

    public RobotConfig(DifferentialDriveKinematics width, double length, double maxSpeed, double maxAcceleration) {
        this.width = width;
        this.length = length;
        this.maxSpeed = maxSpeed;
        this.maxAcceleration = maxAcceleration;
    }

    public DifferentialDriveKinematics getWidth() {
        return width;
    }

    public double getLength() {
        return length;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }
}

