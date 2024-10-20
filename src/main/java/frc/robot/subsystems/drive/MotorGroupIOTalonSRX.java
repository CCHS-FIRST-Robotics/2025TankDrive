package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public class MotorGroupIOTalonSRX implements MotorGroupIO {
    private TalonSRX main_Motor;
    private TalonSRX follower_Motor;
    int encoderTicks;


    public MotorGroupIOTalonSRX(int id1, int id2, boolean inverse_Motors){
    encoderTicks = 4096;
    main_Motor = new TalonSRX(id1);
    follower_Motor = new TalonSRX(id2);
    main_Motor.configFactoryDefault();
        main_Motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0); //only one perevery two motors so hsould all this be done on left and right motor groups and iff so would it need seperate pid for each


        main_Motor.config_kP(0, 1, 0);
		main_Motor.config_kI(0, 0, 0);
		main_Motor.config_kD(0, 0, 0);
        main_Motor.config_kF(0, 0, 0);
       
        if (inverse_Motors){
        main_Motor.setInverted(true); 
      
        } 
        else{
        main_Motor.setInverted(false); 
      
        }

        main_Motor.setSensorPhase(true); // if this need to be switched you will know run robot look at advantage scope
        main_Motor.setSelectedSensorPosition(main_Motor.getSensorCollection().getPulseWidthPosition(), 0, 0);
        follower_Motor.follow(main_Motor);
    }

    
    public void setVoltage(Measure<Voltage> volts) {
        main_Motor.set(TalonSRXControlMode.PercentOutput, volts.in(Volts) / 12);

    }

    // @Override
    // public void setPosition(Measure<Angle> degress){
    //     motor.set(TalonSRXControlMode.Position, degress.in(Rotations));
    // }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity){ 
        main_Motor.set(TalonSRXControlMode.Velocity, velocity.in(RotationsPerSecond) * 0.1 * encoderTicks);
    
        
    }
    @Override
    public Measure<Angle> getSensorPosition(){
        return Rotations.of(main_Motor.getSelectedSensorPosition() / encoderTicks);
    }

     @Override
    public Measure<Velocity<Angle>> getSensorVelocity(){
        return RotationsPerSecond.of((main_Motor.getSelectedSensorVelocity() / encoderTicks) * 10);
    }
        
    
          

    @Override
    public void updateInputs(MotorGroupIOInputs inputs) {
        inputs.main_Motor_Current = Amps.of(main_Motor.getStatorCurrent());
        inputs.main_Motor_Voltage = Volts.of(main_Motor.getBusVoltage());
        inputs.main_Motor_Position = Degrees.of(main_Motor.getSelectedSensorPosition());
        inputs.main_Motor_Velocity = RotationsPerSecond.of((main_Motor.getSelectedSensorVelocity() / encoderTicks) / 2);
        inputs.main_Motor_Temperature = Celsius.of(main_Motor.getTemperature());

        inputs.follower_Motor_Current = Amps.of(follower_Motor.getStatorCurrent());
        inputs.follower_Motor_Voltage = Volts.of(follower_Motor.getBusVoltage());
        inputs.follower_Motor_Position = Degrees.of(follower_Motor.getSelectedSensorPosition());
        inputs.follower_Motor_motorVelocity = RotationsPerSecond.of(follower_Motor.getSelectedSensorVelocity());
        inputs.follower_Motor_Temperature = Celsius.of(follower_Motor.getTemperature());
    }
}



    

