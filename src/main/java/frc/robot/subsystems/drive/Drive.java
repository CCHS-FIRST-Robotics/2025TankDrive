package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import org.littletonrobotics.junction.Logger;



import edu.wpi.first.units.*;
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
        this.distance_pidController.setTolerance(1);
       
        this.odometry = new DifferentialDriveOdometry(
            gyroInputs.connected ? new Rotation2d(gyroInputs.heading): new Rotation2d(),
            lInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE.in(Meters),
            rInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE.in(Meters),
            new Pose2d(0, 0, new Rotation2d())
           

        );
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        lIO.updateInputs(lInputs);
        rIO.updateInputs(rInputs);
        switch (Constants.MODE){
        case REAL:{
        robotPose2d = odometry.update(
            gyroInputs.connected ? new Rotation2d(gyroInputs.heading): new Rotation2d(),
            lInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE.in(Meters), 
            rInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE.in(Meters)
        );
        break;
        }
        case SIM:{
            double leftMeters = lInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE.in(Meters);
            double rightMeters = rInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE.in(Meters);

            double averageMeters = (leftMeters + rightMeters) / 2;

            
            Rotation2d simRotation = new Rotation2d(averageMeters / Constants.TRACK_WIDTH.in(Meters));
            robotPose2d = odometry.update(
            simRotation,
            lInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE.in(Meters), 
            rInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE.in(Meters)
        );
        break;
        }
            default:
                break;
    }

        Logger.processInputs("Gyro ", gyroInputs);
        Logger.recordOutput("RobotPose2D", robotPose2d);
        Logger.processInputs("Left side ", lInputs);
        Logger.processInputs("Right side ", rInputs);
    }

    public void setVoltage(Measure<Voltage> lVolts, Measure<Voltage> rVolts){
        lIO.setVoltage(lVolts);
        rIO.setVoltage(rVolts);
    }

    
    public void setDriveBrakeMode(boolean enable) {
        lIO.setDriveBrakeMode(enable);
        rIO.setDriveBrakeMode(enable);
    }

    public Measure<Angle> getLeftRotations(){
        return lInputs.motor1Position;
    }

      public Measure<Angle> getRightRotations(){
        return rInputs.motor1Position;
    }


    public boolean goForward(Measure<Angle> target_angle, Measure<Velocity<Distance>> Mps, Measure<Angle> target_rotations){
       
        double driverr = (-((lInputs.motor1Position.in(Rotations) + rInputs.motor1Position.in(Rotations)) / 2) - target_rotations.in(Rotations)) * Constants.WHEEL_CIRCUMFERENCE.in(Meters);
        double turnerr =  target_angle.in(Degrees) - gyroInputs.heading;

        double turnpidOutput = turn_pidController.calculate(turnerr);
        double drivepidOutput = MathUtil.clamp(distance_pidController.calculate(driverr), -Mps.in(MetersPerSecond), Mps.in(MetersPerSecond));

        ChassisSpeeds speeds = new ChassisSpeeds(
            drivepidOutput,
            0, 
            turnpidOutput 
        );
        setVelocity(speeds);
        Logger.recordOutput("drive/turn pid output", turnpidOutput );
        Logger.recordOutput("drive/drive pid output ", drivepidOutput );
        Logger.recordOutput("drive/target meters ", target_rotations.in(Rotations) * Constants.WHEEL_CIRCUMFERENCE.in(Meters));
        Logger.recordOutput("drive/current rotations ",  ((lInputs.motor1Position.in(Rotations) + rInputs.motor1Position.in(Rotations)) / 2) * Constants.WHEEL_CIRCUMFERENCE.in(Meters));
        Logger.recordOutput("drive/target rotations ", target_rotations );
        Logger.recordOutput("drive/ speed(MPS)", lInputs.motor1Velocity.in(RotationsPerSecond) * Constants.WHEEL_CIRCUMFERENCE.in(Meters));
        if(Math.abs(driverr) <= .3 && Math.abs(turnerr) <= 2 ){
            return true;
        }
        return false;
        }


    

    


    public void setVelocity(ChassisSpeeds speeds){
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        double leftRadiansPerSecond = wheelSpeeds.leftMetersPerSecond / Constants.WHEEL_RADIUS.in(Meters) / Constants.GEAR_RATIO;
        double rightRadiansPerSecond = wheelSpeeds.rightMetersPerSecond / Constants.WHEEL_RADIUS.in(Meters) / Constants.GEAR_RATIO;
        
        lIO.setVelocity(RadiansPerSecond.of(leftRadiansPerSecond));
        rIO.setVelocity(RadiansPerSecond.of(rightRadiansPerSecond));
    }

    public double getHeading(){
        return gyroInputs.heading;
    }
}