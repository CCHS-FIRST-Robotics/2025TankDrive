package frc.robot.subsystems.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.*;

public interface GyroIO {

    @AutoLog
    public static class GyroIOInputs {
        public boolean imu_Connected = false;
        public Measure<Angle> gyroYawRad = Radians.of(0.0);
        public Measure<Angle> gyroPitchRad = Radians.of(0.0);
        public Measure<Angle> gyroRollRad = Radians.of(0.0);
        public Measure<Angle> gyroYawDeg = Degrees.of(0.0);
        public Measure<Angle> gyroPitchDeg = Degrees.of(0.0);
        public Measure<Angle> gyroRollDeg = Degrees.of(0.0);
        public Measure<Angle> turn_times = Rotations.of(0.0);
        public Measure<Velocity<Distance>> xAccelMps = MetersPerSecond.of(0.0);
        public Measure<Velocity<Distance>> yAccelMps = MetersPerSecond.of(0.0);
        public Measure<Velocity<Distance>> xVelocityMps = MetersPerSecond.of(0.0);
        public Measure<Velocity<Distance>> yVelocityMps = MetersPerSecond.of(0.0);
    }

    public default void updateInputs(GyroIOInputs inputs){

    }


    public default Rotation2d getPitch(){
        return new Rotation2d();
    }

    public default Rotation2d get(){
        return new Rotation2d();
    }

    public default Rotation2d getYaw(){
        return new Rotation2d();
    }

    public default Rotation2d getRoll(){
        return new Rotation2d();
    }
    public default void resetImu(){
        
    }









}

    
