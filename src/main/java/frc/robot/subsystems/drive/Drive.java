package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static edu.wpi.first.units.Units.*;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import frc.robot.Constants;

import frc.robot.subsystems.Motors.*;



public class Drive extends SubsystemBase {
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
    private final DifferentialDriveOdometry odometry;
    private Pose2d currentPose = new Pose2d(0, 0, new Rotation2d());
    private final MotorGroupIOSparkSRX Left_io = new MotorGroupIOSparkSRX(Constants.LEFT_ID_1, Constants.LEFT_ID_2, false);
     private final MotorGroupIOSparkSRX Right_io = new MotorGroupIOSparkSRX(Constants.RIGHT_ID_1, Constants.RIGHT_ID_2, true);
    private final MotorGroup leftMotors = new MotorGroup(Left_io, "Left Motors");
    private final MotorGroup rightMotors = new MotorGroup(Right_io, "Right Motors");
    private final Gyro NavX = new Gyro(); 



   

    public Drive() {
        odometry = new DifferentialDriveOdometry(NavX.getYaw(), leftMotors.distanceTraveled(), rightMotors.distanceTraveled(), currentPose);
    }




    public Pose2d getPose() {
        return currentPose;
    }

    public DifferentialDriveKinematics getKinematics(){
        return kinematics;
    }




 
    public Measure<Distance> leftDistanceTraveled(){
        return leftMotors.distanceTraveled();

}

    public void setVoltage(Measure<Voltage> volts){
        leftMotors.setVoltage(volts);
        rightMotors.setVoltage(volts);
    }


    public void setVelocity(ChassisSpeeds speeds){
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        Measure<Velocity<Angle>> leftRadiansPerSecond = RadiansPerSecond.of(wheelSpeeds.leftMetersPerSecond / (Constants.WHEEL_CIRCUMFERENCE.in(Meters) * Constants.GEAR_RATIO));
        Measure<Velocity<Angle>> rightRadiansPerSecond = RadiansPerSecond.of(wheelSpeeds.rightMetersPerSecond / (Constants.WHEEL_CIRCUMFERENCE.in(Meters) * Constants.GEAR_RATIO));

        leftMotors.setVelocity(leftRadiansPerSecond);
        rightMotors.setVelocity(rightRadiansPerSecond);
    }

    public Measure<Distance> rightDistanceTravled(){
        return rightMotors.distanceTraveled();

    }

    public void Stop(){
        leftMotors.setVoltage(Volts.of(0.0));
        rightMotors.setVoltage(Volts.of(0.0));
    }
        

    @Override
    public void periodic() {
        currentPose = odometry.update(NavX.getYaw(), leftMotors.distanceTraveled().in(Meters), rightMotors.distanceTraveled().in(Meters));
        NavX.updateInputs(); 
       
    }



}
