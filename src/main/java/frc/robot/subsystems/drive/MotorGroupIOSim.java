package frc.robot.subsystems.drive;

import frc.robot.Constants;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MotorGroupIOSim implements MotorGroupIO {
    private final DCMotorSim motor1;
  
public MotorGroupIOSim(){


  

    motor1 = new DCMotorSim(
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
            * Constants.ENCODERTICKS
            * 0.1
        );
    }

    @Override
    public void updateInputs(MotorGroupIOInputs inputs) {
        inputs.main_Motor_Current = motor2.getStatorCurrent();
        inputs.main_Motor_Voltage = motor2.getMotorOutputVoltage();
        inputs.main_Motor_Position = motor2.getSelectedSensorPosition();
        inputs.main_Motor_Velocity = motor2.getSelectedSensorVelocity();
        inputs.main_Motor_Temperature = motor2.getTemperature();

        inputs.follower_Motor_Current = motor2.getStatorCurrent();
        inputs.follower_Motor_Voltage = motor2.getMotorOutputVoltage();
        inputs.follower_Motor_Position = motor2.getSelectedSensorPosition();
        inputs.follower_Motor_motorVelocity = motor2.getSelectedSensorVelocity();
        inputs.follower_Motor_Temperature = motor2.getTemperature();
    }
}
