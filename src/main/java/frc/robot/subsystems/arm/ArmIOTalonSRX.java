package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;


public class ArmIOTalonSRX implements ArmIO {
    private TalonSRX motor;
    
    public ArmIOTalonSRX(int id){
        motor = new TalonSRX(id);
        motor.configFactoryDefault();
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

        motor.config_kP(0, 1, 0);
		motor.config_kI(0, 0, 0);
		motor.config_kD(0, 0, 0);
        motor.config_kF(0, 0, 0);

        motor.setInverted(false);
        motor.setSensorPhase(true);
        motor.setSelectedSensorPosition(motor.getSensorCollection().getPulseWidthPosition(), 0, 0);
    }

    

    @Override
    public void setPosition(Measure<Angle> position){
         motor.set(TalonSRXControlMode.Position, position.in(Rotations));
    }

    // public void setVelocity(double velocity){
    //     motor.set(TalonSRXControlMode.Velocity, velocity);
    // }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.motorCurrent = motor.getStatorCurrent();
        inputs.motorVoltage = motor.getBusVoltage();
        inputs.motorPosition = motor.getSelectedSensorPosition();
        inputs.motorVelocity = motor.getSelectedSensorVelocity();
        inputs.motorTemperature = motor.getTemperature();
    }
}