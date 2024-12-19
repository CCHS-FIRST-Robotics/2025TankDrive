//maybe subtract gyro values instead of gyro.reset()
package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.Constants;

public class Drive extends SubsystemBase{
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final DifferentialDriveOdometry odometry;
    private Pose2d robotPose2d = new Pose2d();
    
    PIDController headingController = new PIDController(0.3, 0, 0.0);
    boolean piding = false;
    double targetHeading = 0;
    ChassisSpeeds speeds;
    
    PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

    private final DriveSideIO lIO, rIO;
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
    private final DriveSideIOInputsAutoLogged lInputs = new DriveSideIOInputsAutoLogged();
    private final DriveSideIOInputsAutoLogged rInputs = new DriveSideIOInputsAutoLogged();

    public Drive(GyroIO gyroIO, DriveSideIO leftIO, DriveSideIO rightIO) {
        this.lIO = leftIO;
        this.rIO = rightIO;
        this.gyroIO = gyroIO;
        this.odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0, new Pose2d(0, 0, new Rotation2d()));

        ReplanningConfig replanningConfig = new ReplanningConfig();


        AutoBuilder.configureLTV(() -> this.robotPose2d, this::resetPose, () -> (this.speeds), this::setVelocity, 0.2, 
        replanningConfig, () -> false, this);
        
    }

    private void resetPose(Pose2d pose) {
        this.robotPose2d = new Pose2d();
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        robotPose2d = odometry.update(gyroInputs.connected ? gyroInputs.rotation2D: new Rotation2d(),lInputs.distanceTraveled, rInputs.distanceTraveled);
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

    public void setVelocity(ChassisSpeeds chassisSpeeds){
        
        if (chassisSpeeds.omegaRadiansPerSecond == 0 && chassisSpeeds.vxMetersPerSecond != 0 && !piding) {
            targetHeading = gyroInputs.heading;
            piding = true;
        } 
        
        if (chassisSpeeds.omegaRadiansPerSecond != 0) {
            piding = false;
        }

        if(piding){
            chassisSpeeds = new ChassisSpeeds(
                chassisSpeeds.vxMetersPerSecond, 
                chassisSpeeds.vyMetersPerSecond, 
                chassisSpeeds.omegaRadiansPerSecond - headingController.calculate(gyroInputs.heading, targetHeading)
            );
        } 
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

        speeds = chassisSpeeds;

        lIO.setVelocity(leftMotorVelocity); // should be 88.83 rotations per second
        rIO.setVelocity(rightMotorVelocity);

        
    }

    public double getLeftEncoderDistance() {
        return lInputs.distanceTraveled;
    }

    public double getRightEncoderDistance() {
        return rInputs.distanceTraveled;
    }

    public Rotation2d getGyroRotation() {
        return gyroInputs.rotation2D;
    }
}