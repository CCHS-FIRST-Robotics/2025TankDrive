package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import edu.wpi.first.units.*;

public class DriveSideIOSim implements DriveSideIO {
    private final DCMotorSim motor;
    
    public DriveSideIOSim(){
        motor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(0, 0), 
            DCMotor.getCIM(1), // or just divide kA by 2, since kT * torque = volts and kT = kA
            Constants.GEAR_RATIO
        );
    }
    
    @Override
    public void setVoltage(Measure<Voltage> volts) {
        motor1.set(TalonSRXControlMode.PercentOutput, volts.in(Volts) / 12);
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity){
        motor1.set( // this function takes in encoder ticks per 0.1 seconds
            TalonSRXControlMode.Velocity, 
            velocity.in(RotationsPerSecond) 
            * encoderTicks
            * 0.1
        );
    }

    @Override
    public void updateInputs(DriveSideIOInputs inputs) {
        inputs.motor1Current = motor2.getStatorCurrent();
        inputs.motor1Voltage = motor2.getMotorOutputVoltage();
        inputs.motor1Position = motor2.getSelectedSensorPosition();
        inputs.motor1Velocity = motor2.getSelectedSensorVelocity();
        inputs.motor1Temperature = motor2.getTemperature();

        inputs.motor2Current = motor2.getStatorCurrent();
        inputs.motor2Voltage = motor2.getMotorOutputVoltage();
        inputs.motor2Position = motor2.getSelectedSensorPosition();
        inputs.motor2Velocity = motor2.getSelectedSensorVelocity();
        inputs.motor2Temperature = motor2.getTemperature();
    }
}