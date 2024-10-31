package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.*;
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
    
    private final DriveSideIO lIO, rIO;
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
    private final DriveSideIOInputsAutoLogged lInputs = new DriveSideIOInputsAutoLogged();
    private final DriveSideIOInputsAutoLogged rInputs = new DriveSideIOInputsAutoLogged();

    public Drive(GyroIO gyroIO, DriveSideIO leftIO, DriveSideIO rightIO) {
        this.lIO = leftIO;
        this.rIO = rightIO;
        this.gyroIO = gyroIO;

        double leftUpdatedPos = 0; //but make sure encoders reset every deploy
        double rightUpdatedPos = 0; //but make sure encoders reset every deploy

        this.odometry = new DifferentialDriveOdometry(gyroInputs.rotation2D, leftUpdatedPos, rightUpdatedPos, new Pose2d(0, 0, new Rotation2d()));
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        lIO.updateInputs(lInputs);
        rIO.updateInputs(rInputs);

        double leftUdatedPos = lInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE;
        double rightUpdatedPos = rInputs.motor1Position.in(Rotations) * Constants.GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE;

        robotPose2d = odometry.update(gyroInputs.connected ? gyroInputs.rotation2D: new Rotation2d(), leftUdatedPos, rightUpdatedPos);

        Logger.processInputs("Gyro ", gyroInputs);
        Logger.recordOutput("RobotPose2D", robotPose2d);
        Logger.processInputs("Left side ", lInputs);
        Logger.processInputs("Right side ", rInputs);
    }

    public void setVoltage(Measure<Voltage> lVolts, Measure<Voltage> rVolts){
        lIO.setVoltage(lVolts);
        rIO.setVoltage(rVolts);
    }

    public void setVelocity(ChassisSpeeds speeds){
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        double leftRadPerSecond = wheelSpeeds.leftMetersPerSecond / Constants.WHEEL_RADIUS / Constants.GEAR_RATIO;
        double rightRadPerSecond = wheelSpeeds.rightMetersPerSecond / Constants.WHEEL_RADIUS / Constants.GEAR_RATIO;
        
        lIO.setVelocity(RadiansPerSecond.of(leftRadPerSecond));
        rIO.setVelocity(RadiansPerSecond.of(rightRadPerSecond));
    }
}