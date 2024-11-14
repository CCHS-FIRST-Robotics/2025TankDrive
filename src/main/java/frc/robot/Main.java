// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do: 
 * 
 * ! check the track width again
 * ! check sensorphase with setvoltage
 * check the gear ratio by seeing if at 12V the encoder reads 6.9 (88.83/Gear_ratio) rotations per second
 * tune P
 * 
 * check gyro logging
 * 
 * add twist support for sim
 */

public final class Main {
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}