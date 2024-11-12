package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import frc.robot.Constants;

public class DriveSideIOTalonSRX implements DriveSideIO {
    private final TalonSRX motor1, motor2;
    private final PIDController PID;
    private final SimpleMotorFeedforward F;
    int encoderTicks = 4096;

    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0;
    private final double kS = 0;
    private final double kV = 0.135; // at 88.83 rotations per second, output 12 volts
    private final double kA = 0;

    Measure<Velocity<Angle>> currentSetpoint = RotationsPerSecond.of(0);
    
    public DriveSideIOTalonSRX(int id1, int id2, boolean isInverted){
        motor1 = new TalonSRX(id1);
        motor2 = new TalonSRX(id2);
        PID = new PIDController(kP, kI, kD);
        F = new SimpleMotorFeedforward(kS, kV, kA);

        motor1.configFactoryDefault();
        motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        motor1.setSelectedSensorPosition(0);

        // motor1.config_kP(0, kP, 0);
		// motor1.config_kI(0, kI, 0);
		// motor1.config_kD(0, kD, 0);
        // motor1.config_kF(0, kF, 0);

        motor1.setInverted(isInverted);
        motor2.follow(motor1);
        motor2.setInverted(InvertType.FollowMaster);
    }
    
    @Override
    public void setVoltage(Measure<Voltage> volts) {
        motor1.set(TalonSRXControlMode.PercentOutput, volts.in(Volts) / 12);
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity){
        // motor1.set( // this function takes in encoder ticks per 0.1 seconds
        //     TalonSRXControlMode.Velocity, 
        //     velocity.in(RotationsPerSecond) * encoderTicks * 0.1
        // );

        double currentVelocityRotationsPerSecond = motor1.getSelectedSensorVelocity() * 10 / 4096;

        this.setVoltage(Volts.of(
            PID.calculate(currentVelocityRotationsPerSecond, velocity.in(RotationsPerSecond))
            + F.calculate(velocity.in(RotationsPerSecond))
        ));
        currentSetpoint = velocity;
    }

    @Override
    public void updateInputs(DriveSideIOInputs inputs) {
        inputs.currentSetpoint = currentSetpoint;
        inputs.distanceTraveled = Meters.of((motor1.getSelectedSensorPosition() / 4096) * Constants.WHEEL_RADIUS * Constants.GEAR_RATIO);

        inputs.motor1Current = Amps.of(motor1.getStatorCurrent());
        inputs.motor1Voltage = Volts.of(motor1.getMotorOutputVoltage());
        inputs.motor1Position = Rotations.of(motor1.getSelectedSensorPosition() / 4096);
        inputs.motor1Velocity = RotationsPerSecond.of(motor1.getSelectedSensorVelocity() * 10 / 4096);
        inputs.motor1Temperature = Celsius.of(motor1.getTemperature());
    }
}