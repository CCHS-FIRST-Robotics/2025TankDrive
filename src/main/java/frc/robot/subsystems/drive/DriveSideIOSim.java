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

    private final double kP = 10;
    private final double kI = 0;
    private final double kD = 0;
    private final double kS = 0;
    private final double kV = 2.54 * Constants.WHEEL_RADIUS / Constants.GEAR_RATIO;
    private final double kA = 0.27 * Constants.WHEEL_RADIUS / Constants.GEAR_RATIO;

    Measure<Voltage> appliedVolts = Volts.of(0);
    Measure<Velocity<Angle>> currentSetpoint = RotationsPerSecond.of(0);
    
    public DriveSideIOSim(){
        motor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(kV, kA),
            DCMotor.getCIM(2), // or just divide kA by 2, since kT * torque = volts and kT = kA
            Constants.GEAR_RATIO
        );
        PID = new PIDController(kP, kI, kD);
        F = new SimpleMotorFeedforward(kS, kV, kA);
    }
    
    @Override
    public void setVoltage(Measure<Voltage> volts) {
        motor.setInputVoltage(volts.in(Volts));
        appliedVolts = volts;
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity){
        double volts = PID.calculate(motor.getAngularVelocityRadPerSec(), velocity.in(RadiansPerSecond))
                       + F.calculate(currentSetpoint.in(RadiansPerSecond));
        
        this.setVoltage(Volts.of(volts));
        currentSetpoint = velocity;
    }

    @Override
    public void updateInputs(DriveSideIOInputs inputs) {
        motor.update(Constants.PERIOD);

        inputs.currentSetpoint = currentSetpoint.in(RadiansPerSecond);

        inputs.motor1Current = motor.getCurrentDrawAmps();
        inputs.motor1Voltage = appliedVolts.in(Volts);
        inputs.motor1Position = motor.getAngularPositionRad();
        inputs.motor1Velocity = motor.getAngularVelocityRadPerSec();
        inputs.motor1Temperature = 0;

        inputs.motor2Current = motor.getCurrentDrawAmps();
        inputs.motor2Voltage = appliedVolts.in(Volts);
        inputs.motor2Position = motor.getAngularPositionRad();
        inputs.motor2Velocity = motor.getAngularVelocityRadPerSec();
        inputs.motor2Temperature = 0;
    }
}