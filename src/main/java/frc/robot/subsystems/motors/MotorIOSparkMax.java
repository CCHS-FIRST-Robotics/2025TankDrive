package frc.robot.subsystems.motors;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.units.*;

public class MotorIOSparkMax implements MotorIO {
    private final CANSparkMax motor;
    private final AbsoluteEncoder encoder;
    private final SparkPIDController PIDF;

    public MotorIOSparkMax(int id){
        motor = new CANSparkMax(id, MotorType.kBrushed);
        encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        PIDF = motor.getPIDController();

        PIDF.setP(10, 0);
        PIDF.setD(0, 0);
        PIDF.setI(0, 0);
        PIDF.setFF(0, 0);
        PIDF.setFeedbackDevice(encoder);

        motor.setCANTimeout(500);
        motor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus5, // returns encoder position frame
            20
        );
        motor.setSmartCurrentLimit(30);
        motor.enableVoltageCompensation(12);
        motor.setIdleMode(IdleMode.kBrake);
        encoder.setAverageDepth(2);
        motor.setCANTimeout(0);
        motor.burnFlash();
    }

    @Override
    public void setVoltage(Measure<Voltage> volts){
        motor.setVoltage(volts.in(Volts));
    }

    @Override
    public void setPosition(Measure<Angle> position){ // ! hasn't worked yet
        PIDF.setReference(
            position.in(Rotations),
            CANSparkMax.ControlType.kPosition,
            0
        );
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorVoltage = motor.getBusVoltage();
        inputs.motorPosition = encoder.getPosition();
        inputs.motorVelocity = encoder.getVelocity();
        inputs.motorTemperature = motor.getMotorTemperature();
    }
}