package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.units.*;

public class DriveSideIOTalonSRX implements DriveSideIO {
    private final TalonSRX motor1, motor2;
    int encoderTicks = 4096;

    Measure<Velocity<Angle>> currentSetpoint = RotationsPerSecond.of(0);
    
    public DriveSideIOTalonSRX(int id1, int id2, boolean isInverted){
        motor1 = new TalonSRX(id1);
        motor2 = new TalonSRX(id2);

        motor1.configFactoryDefault();
        motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

        motor1.config_kP(0, 1, 0);
		motor1.config_kI(0, 0, 0);
		motor1.config_kD(0, 0, 0);
        motor1.config_kF(0, 1, 0); // ! should do whatever the docs say

        motor1.setInverted(isInverted);
        // motor1.setSensorPhase(true); // ! keep in mind

        motor2.follow(motor1);
    }
    
    @Override
    public void setVoltage(Measure<Voltage> volts) {
        motor1.set(TalonSRXControlMode.PercentOutput, volts.in(Volts) / 12);
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity){
        motor1.set( // this function takes in encoder ticks per 0.1 seconds
            TalonSRXControlMode.Velocity, 
            velocity.in(RotationsPerSecond) * encoderTicks * 0.1
        );

        currentSetpoint = velocity;
    }

    @Override
    public void updateInputs(DriveSideIOInputs inputs) {
        inputs.currentSetpoint = currentSetpoint.in(RotationsPerSecond) * encoderTicks * 0.1;

        inputs.motor1Current = motor1.getStatorCurrent();
        inputs.motor1Voltage = motor1.getMotorOutputVoltage();
        inputs.motor1Position = motor1.getSelectedSensorPosition();
        inputs.motor1Velocity = motor1.getSelectedSensorVelocity();
        inputs.motor1Temperature = motor1.getTemperature();

        inputs.motor2Current = motor2.getStatorCurrent();
        inputs.motor2Voltage = motor2.getMotorOutputVoltage();
        inputs.motor2Position = motor2.getSelectedSensorPosition();
        inputs.motor2Velocity = motor2.getSelectedSensorVelocity();
        inputs.motor2Temperature = motor2.getTemperature();
    }
}