package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;


import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public interface MotorGroupIO {
    @AutoLog
    public static class MotorIOInputs {
        public Measure<Current> main_Motor_Current;
        public Measure<Voltage> main_Motor_Voltage;
        public Measure<Angle> main_Motor_Position;
        public Measure<Velocity<Angle>> main_Motor_Velocity;
        public Measure<Temperature> main_Motor_Temperature;

        public Measure<Current> follower_Motor_Current;
        public Measure<Voltage> follower_Motor_Voltage;
        public Measure<Angle> follower_Motor_Position;
        public Measure<Velocity<Angle>> follower_Motor_motorVelocity;
        public Measure<Temperature> follower_Motor_Temperature;
    }

    public default void setVoltage(Measure<Voltage> volts) {
    }

    // public default void setPosition(Measure<Angle> degress){
    // }

    public default void updateInputs(MotorIOInputs inputs) {
    }

    public default void setVelocity(Measure<Velocity<Angle>> speed){ //Meters per second
    }
    
    public default Measure<Angle> getSensorPosition(){
        return Degrees.of(0);
    }
    public default Measure<Velocity<Angle>> getSensorVelocity(){
        return RotationsPerSecond.of(0.0);
    }
    




}