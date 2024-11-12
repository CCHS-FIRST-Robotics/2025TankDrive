// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import java.util.ArrayList;

public final class Constants {
    public static final double PERIOD = 0.02; //20 ms
    public static final double ANALOG_DEADZONE = 0.1;
    public static final double JOYSTICK_EXPONENT = 3.5;
    public static enum Mode {REAL, SIM, REPLAY}
    public static final Mode MODE = Mode.REAL;

    public static final int CONTROLLER_PORT = 0;

    public static final int TALONSRX_ID_1 = 23;
    public static final int TALONSRX_ID_2 = 2;
    public static final int TALONSRX_ID_3 = 10;
    public static final int TALONSRX_ID_4 = 11;

    public static final double TRACK_WIDTH = 0.66; //meters
    public static final double WHEEL_RADIUS = 0.0762; //meters
    public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;
    public static final double GEAR_RATIO = 1/10.71;
    public static final double MAX_SPEED = 3;

    /*
    public static class AutoPathConstants {        
        // arraylists for paths
        public static final ArrayList<String> twoStraight = new ArrayList<String>();

        // file names for paths
        public static final String STRAIGHT1 = "Forward1Meter";

        public static final double MAX_MOVE_TIME = 1.0;

        
        static {
            twoStraight.add(STRAIGHT1);
        }
        
    }
    */
}