package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.units.*;
import frc.robot.Constants;

public class DriveSideIOSim implements DriveSideIO {
    private final DCMotorSim motor;
    private final PIDController PID;
    private final SimpleMotorFeedforward F;
    Measure<Voltage> appliedVolts = Volts.of(0);
    Measure<Velocity<Angle>> currentSetpoint = RotationsPerSecond.of(0);
    
    public DriveSideIOSim(){
        motor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem( // volts per RPM
                2.54 * Constants.GEAR_RATIO * 60 / Constants.WHEEL_CIRCUMFERENCE,
                0.27 * Constants.GEAR_RATIO * 60 / Constants.WHEEL_CIRCUMFERENCE
            ), 
            DCMotor.getCIM(1), // or just divide kA by 2, since kT * torque = volts and kT = kA
            Constants.GEAR_RATIO
        );
        PID = new PIDController(1, 0, 0);
        F = new SimpleMotorFeedforward(0, 2.54, 0.27);
    }
    
    @Override
    public void setVoltage(Measure<Voltage> volts) {
        motor.setInputVoltage(volts.in(Volts));
        appliedVolts = volts;
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity){
        double volts = PID.calculate(motor.getAngularVelocityRPM() / 60, velocity.in(RotationsPerSecond))
                       + F.calculate(currentSetpoint.in(RotationsPerSecond));
        
        this.setVoltage(Volts.of(volts));
        currentSetpoint = velocity;
    }

    @Override
    public void updateInputs(DriveSideIOInputs inputs) {
        motor.update(Constants.PERIOD);

        inputs.motor1Current = motor.getCurrentDrawAmps();
        inputs.motor1Voltage = appliedVolts.in(Volts);
        inputs.motor1Position = motor.getAngularPositionRotations();
        inputs.motor1Velocity = motor.getAngularVelocityRPM() / 60;
        inputs.motor1Temperature = 0;

        inputs.motor2Current = motor.getCurrentDrawAmps();
        inputs.motor2Voltage = appliedVolts.in(Volts);
        inputs.motor2Position = motor.getAngularPositionRotations();
        inputs.motor2Velocity = motor.getAngularVelocityRPM() / 60;
        inputs.motor2Temperature = 0;
    }
}