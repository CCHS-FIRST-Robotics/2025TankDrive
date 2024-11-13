// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 * 
 * ! check the track width again
 * 
 * check the gear ratio and wheel radius
 * when it's fully forward, motor is spinning at 88.83 rps, multiply by gear ratio and radius for meters per second
 * then change max speed
 * 
 * encoder measures wheel rotation
 * check if the gyro is accurately logging
 * 
 * add twist support for sim
 * standardize logged units for everything
 */

public final class Main {
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}