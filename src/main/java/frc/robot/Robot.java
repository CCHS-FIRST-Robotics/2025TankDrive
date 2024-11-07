// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.*;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
    RobotContainer robotContainer;
    double count;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        count = 0;


        Logger.recordMetadata("ProjectName", "2025TankDrive");

        switch (Constants.MODE) {
            case REAL:
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIM:
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;
        }

        Logger.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        double volts = RobotController.getBatteryVoltage();
        if(volts <= 11.5){
            count++;
        }
        if (volts >= 11.5){
            count = 0;
        }
        if(count >= 50){
            System.out.print((char)7);
        }
        

    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
      CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}