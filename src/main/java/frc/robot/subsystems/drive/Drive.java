package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
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
    private final PIDController turn_pidController;
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
        this.turn_Kp = .08;
        this.turn_Ki = 0.0;
        this.turn_Kd = 0.01;
        this.turn_pidController = new PIDController(turn_Kp, turn_Ki, turn_Kd);
       
        this.odometry = new DifferentialDriveOdometry(
            gyroInputs.connected ? new Rotation2d(gyroInputs.heading): new Rotation2d(),
            lInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE,
            rInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE,
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
            lInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE, 
            rInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE
        );
        }
        case SIM:{
            double leftMeters = lInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE;
            double rightMeters = rInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE;

            double averageMeters = (leftMeters + rightMeters) / 2;

            
            Rotation2d simRotation = new Rotation2d(averageMeters / Constants.TRACK_WIDTH);
            robotPose2d = odometry.update(
            simRotation,
            lInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE, 
            rInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE
        );
            
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


    public boolean goForward(Measure<Angle> angle, Measure<Velocity<Distance>> Mps, Measure<Distance> distance){
        double rotations = (distance.in(Meters) / Constants.WHEEL_CIRCUMFERENCE);
        if ((lInputs.motor1Position.in(Rotations) + rInputs.motor1Position.in(Rotations)) / 2 >= rotations) {
            return true;
        }
        else{
        double current_Angle = gyroInputs.heading * (Math.PI/180); 
        double err = angle.in(Radians) - current_Angle;
        double pidOutput = turn_pidController.calculate(err);

        ChassisSpeeds speeds = new ChassisSpeeds(
            Mps.in(MetersPerSecond),
            0, 
            pidOutput 
        );
        setVelocity(speeds);
        Logger.recordOutput("drive/err", err);
        Logger.recordOutput("drive/pid output", pidOutput );
        Logger.recordOutput("drive/speed(MPS)", lInputs.motor1Velocity.in(RotationsPerSecond) * Constants.WHEEL_CIRCUMFERENCE);
        return false;
        }


    }


    public void setVelocity(ChassisSpeeds speeds){
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        double leftRadiansPerSecond = wheelSpeeds.leftMetersPerSecond / Constants.WHEEL_RADIUS / Constants.GEAR_RATIO;
        double rightRadiansPerSecond = wheelSpeeds.rightMetersPerSecond / Constants.WHEEL_RADIUS / Constants.GEAR_RATIO;
        
        lIO.setVelocity(RadiansPerSecond.of(leftRadiansPerSecond));
        rIO.setVelocity(RadiansPerSecond.of(rightRadiansPerSecond));
    }

    public double getHeading(){
        return gyroInputs.heading;
    }
}