package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
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
        motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        motor1.setSelectedSensorPosition(0);

        motor1.setInverted(isInverted);

        motor2.follow(motor1);
        motor2.setInverted(InvertType.FollowMaster); //reverse direction of motor 2 to match motor 1 when going forward and back
    }
    
    @Override
    public void setVoltage(Measure<Voltage> volts) {
        motor1.set(TalonSRXControlMode.PercentOutput, volts.in(Volts) / 12);
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity){
        motor1.set(TalonSRXControlMode.Velocity, velocity.in(RotationsPerSecond) * encoderTicks * 0.1); //takes in encoder ticks per 0.1 seconds

        currentSetpoint = velocity;
    }

    @Override
    public void updateInputs(DriveSideIOInputs inputs) {
        inputs.currentSetpoint = currentSetpoint;
        inputs.motorCurrent = Amps.of(motor1.getStatorCurrent());
        inputs.motorVoltage = Volts.of(motor1.getMotorOutputVoltage());
        inputs.motorPosition = Rotations.of(motor1.getSelectedSensorPosition() / 4096);
        inputs.motorVelocity = RotationsPerSecond.of(motor1.getSelectedSensorVelocity() * 10 / 4096);
    }
}