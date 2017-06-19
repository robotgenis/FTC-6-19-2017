package com.pineapplerobotics.velocityvortex.javacodeclasses.resources;

/**
 * Created by krisu on 11/6/2016.
 */

public class RobotVals {
    public static final double BEACON_RIGHT_BACK = 0.95;
    public static final double BEACON_LEFT_BACK = 0;
    public static final double BEACON_RIGHT_FORWARD = 0;
    public static final double BEACON_LEFT_FORWARD = 1;
    public static final double SHOOTER_UP = 0.4;
    public static final double SHOOTER_DOWN = 0.8;
    public static final double SHOOTER_SPEED = 0.20;
    public static final double START_POST = 155;



    public static final double COUNTS_PER_MOTOR_REV_NEVEREST = 280;    // AndyMark NestRev 40 PPR is 280
    public static final double COUNTS_PER_MOTOR_REV_TETRIX = 1440;     // Tetrix Dcmotor PPR is 1440
    public static final double DRIVE_GEAR_REDUCTION = 1.5;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double COUNTS_PER_INCH_NEVEREST = (COUNTS_PER_MOTOR_REV_NEVEREST * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double COUNTS_PER_INCH_TETRIX = (COUNTS_PER_MOTOR_REV_TETRIX) /
            (3 * Math.PI);
    public static final double SPOOL_DIAMETER_MM = 62.3;
    public static final double INCHES_TO_MM = 0.0393701;
    public static final double SPOOL_CIRCUMFERENCE = Math.PI *SPOOL_DIAMETER_MM*INCHES_TO_MM;
    public static final double NUMBER_OF_ROTATIONS_OF_LIFT = 62/SPOOL_CIRCUMFERENCE;

    public static final double NEVEREST_40_RPM = 160;
    public static final double NEVEREST_60_RPM = 105;

    public static final double SECONDS_PER_MINUTE = 60;

    public static final double NEVEREST_40_RPS = NEVEREST_40_RPM/SECONDS_PER_MINUTE;
    public static final double NEVEREST_60_RPS = NEVEREST_60_RPM/SECONDS_PER_MINUTE;

    public static final double NEVEREST_40_SPR = 1/NEVEREST_40_RPS;
    public static final double NEVEREST_60_SPR = 1/NEVEREST_60_RPS;

    public static final double MILLISECONDS_TO_LIFT = NEVEREST_60_SPR*NUMBER_OF_ROTATIONS_OF_LIFT*1000;



}

