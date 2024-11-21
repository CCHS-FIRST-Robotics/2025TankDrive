package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.*;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;

public class Drive extends SubsystemBase{
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final DifferentialDriveOdometry odometry;
    private Pose2d robotPose2d = new Pose2d();
    double turn_Kp;
    double turn_Ki;
    double turn_Kd;
    double distance_Kp;
    double distance_Ki;
    double distance_Kd;
    private final PIDController turn_pidController;
    private final PIDController distance_pidController;
    
    private final DriveSideIO lIO, rIO;
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
    private final DriveSideIOInputsAutoLogged lInputs = new DriveSideIOInputsAutoLogged();
    private final DriveSideIOInputsAutoLogged rInputs = new DriveSideIOInputsAutoLogged();

    public Drive(
        GyroIO gyroIO, 
        DriveSideIO leftIO, 
        DriveSideIO rightIO)
    {
        this.lIO = leftIO;
        this.rIO = rightIO;
        this.gyroIO = gyroIO;
        this.turn_Kp = .1;
        this.turn_Ki = 0.0;
        this.turn_Kd = 0.01;
        this.distance_Kp = 10;
        this.distance_Ki = 0.0;
        this.distance_Kd = 0.00;
        this.turn_pidController = new PIDController(turn_Kp, turn_Ki, turn_Kd);
        this.distance_pidController = new PIDController(distance_Kp, distance_Ki, distance_Kd);
        this.odometry = new DifferentialDriveOdometry(
            new Rotation2d(), 
            0, 
            0, 
            new Pose2d(0, 0, new Rotation2d())
        );
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        robotPose2d = odometry.update(
            gyroInputs.connected ? gyroInputs.rotation2D: new Rotation2d(),
            lInputs.distanceTraveled, 
            rInputs.distanceTraveled
        );
        lIO.updateInputs(lInputs);
        rIO.updateInputs(rInputs);

        Logger.processInputs("Gyro ", gyroInputs);
        Logger.recordOutput("RobotPose2D", robotPose2d);
        Logger.processInputs("Left side ", lInputs);
        Logger.processInputs("Right side ", rInputs);
    }

    public void setVoltage(Measure<Voltage> lVolts, Measure<Voltage> rVolts){
        lIO.setVoltage(lVolts);
        rIO.setVoltage(rVolts);
    }

    public double getLeftRotations(){
        return lInputs.wheelPosition;
    }

    public double getRightRotations(){
        return rInputs.wheelPosition;
    }

    public double getHeading(){
        return gyroInputs.heading;
    }

     public boolean goForward(Measure<Angle> target_angle, Measure<Velocity<Distance>> Mps, Measure<Angle> target_rotations){
       
        double driverr = (target_rotations.in(Rotations) - ((lInputs.motor1Position + rInputs.motor1Position) / 2));
        double turnerr =  target_angle.in(Degrees) - gyroInputs.heading;

        double turnpidOutput = turn_pidController.calculate(turnerr);
        double drivepidOutput = MathUtil.clamp(distance_pidController.calculate(driverr), -Mps.in(MetersPerSecond), Mps.in(MetersPerSecond));

        ChassisSpeeds speeds = new ChassisSpeeds(
            drivepidOutput,
            0, 
            turnpidOutput 
        );
        setVelocity(speeds);
        Logger.recordOutput("drive/drive err", driverr );
        Logger.recordOutput("drive/drive pid output ", drivepidOutput );
        Logger.recordOutput("drive/target meters ", target_rotations.in(Rotations) * Constants.WHEEL_CIRCUMFERENCE.in(Meters));
        Logger.recordOutput("drive/current rotations ",  ((lInputs.motor1Position + rInputs.motor1Position) / 2) * Constants.WHEEL_CIRCUMFERENCE.in(Meters));
        Logger.recordOutput("drive/target rotations ", target_rotations );
        Logger.recordOutput("drive/turn err", turnerr);
        Logger.recordOutput("drive/turn err cutoff", 1);
        Logger.recordOutput("drive/drive err cutoff", .1);
        if(driverr <= .1 && turnerr <= 1 ){
            return true;
        }
        return false;
        }

    public void setVelocity(ChassisSpeeds chassisSpeeds){
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

        Measure<Velocity<Angle>> leftMotorVelocity = RotationsPerSecond.of(
            wheelSpeeds.leftMetersPerSecond 
            / Constants.WHEEL_CIRCUMFERENCE.in(Meters) // to get rotations per second of the wheel
            * Constants.GEAR_RATIO // to get rotations per second of the motor
        );
        Measure<Velocity<Angle>> rightMotorVelocity = RotationsPerSecond.of(
            wheelSpeeds.rightMetersPerSecond 
            / Constants.WHEEL_CIRCUMFERENCE.in(Meters) // to get rotations per second of the wheel
            * Constants.GEAR_RATIO // to get rotations per second of the motor
        );

        lIO.setVelocity(leftMotorVelocity); // should be 88.83 rotations per second
        rIO.setVelocity(rightMotorVelocity);
    }
}