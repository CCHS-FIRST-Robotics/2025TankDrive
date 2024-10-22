// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
    public static final double PERIOD = 0.02;
    public static final double ANALOG_DEADZONE = 0.3;
    public static enum Mode {REAL, SIM, REPLAY}
    public static final Mode MODE = Mode.REAL;

    public static final int CONTROLLER_PORT_1 = 0;

    public static final int TALONSRX_ID_1 = 23;
    public static final int TALONSRX_ID_2 = 2;
    public static final int TALONSRX_ID_3 = 10;
    public static final int TALONSRX_ID_4 = 11;

    public static final double TRACK_WIDTH = 0.66;
    public static final double WHEEL_RADIUS = 0.07;
    public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;
    public static final double GEAR_RATIO = 1;
    public static final double MAX_SPEED = 2;
}