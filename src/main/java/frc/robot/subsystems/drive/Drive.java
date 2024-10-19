package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.kinematics.*;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.*;
import frc.robot.Constants;
import frc.robot.subsystems.drive.tankDrive.DriveSideIOInputsAutoLogged;

public class Drive extends SubsystemBase{
    DriveSideIO lIO, rIO;
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
    DriveSideIOInputsAutoLogged lInputs, rInputs;

    public Drive(DriveSideIO leftIO, DriveSideIO rightIO){
        this.lIO = leftIO;
        this.rIO = rightIO;
    }

    @Override
    public void periodic() {
        lIO.updateInputs(lInputs);
        rIO.updateInputs(rInputs);

        Logger.processInputs("Left side ", lInputs);
        Logger.processInputs("Right side ", rInputs);
    }

    public void setVoltage(Measure<Voltage> lVolts, Measure<Voltage> rVolts){
        lIO.setVoltage(lVolts);
        rIO.setVoltage(rVolts);
    }

    public void setVelocity(ChassisSpeeds speeds){
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        lIO.setVelocity(RadiansPerSecond.of(wheelSpeeds.leftMetersPerSecond));
        rIO.setVelocity(RadiansPerSecond.of(wheelSpeeds.rightMetersPerSecond));
    }
}