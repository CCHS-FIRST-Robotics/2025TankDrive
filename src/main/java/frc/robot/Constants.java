// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.*;

public final class Constants {
    public static double ANALOG_DEADZONE = 0.3;


   
    // CHANGE BEFORE TESTING
    public static Measure<Distance> TRACK_WIDTH = Meters.of(0);
    public static Measure<Distance> WHEEL_DIAMETER = Meters.of(0);
    public static int CONTROLLER_PORT = 0;
    public static int LEFT_ID_1 = 0;
    public static int LEFT_ID_2 = 0;
    public static int RIGHT_ID_1 = 0;
    public static int RIGHT_ID_2 = 0;
    public static int ARM_ID = 0;
    public static double ticksPerRevolution = 4096;
    public static final Measure<Distance> WHEEL_CIRCUMFERENCE =  Meters.of(Math.PI * WHEEL_DIAMETER.in(Meters));
    public static final double GEAR_RATIO = 0;

}
