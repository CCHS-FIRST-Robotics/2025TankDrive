package frc.robot.subsystems.drive.tankDrive;

import edu.wpi.first.wpilibj2.command.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Drive extends SubsystemBase{
    DriveSideIO lIO, rIO;
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

    public void setVoltage(){

    }

    public void setVelocity(ChassisSpeeds speeds){

    }
}