package com.pineapplerobotics.PreseasonSummer2017.ProgressBranch.CustomDrive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public abstract class RobotDrive {
    //Gets Inforamtion
    public abstract String driveType();

    public abstract String driveError();

    public abstract double getDriveScale();

    public abstract boolean isDriveEnabled();

    public abstract boolean isExponentialDrive();

    public abstract double getMaxDriveValue();

    public abstract double getMinDriveValue();

    //Main Drive Function
    //Input should be drive(new DcMotor[]{Left, Right},gamepad1);
    //or drive(new DcMotor[]{LeftFront, RightFront, LeftBack, RightBack},gamepad1);
    //returns true for sucess
    public abstract Boolean gamepadDrive(DcMotor[] Motors, Gamepad gamepad);

    public abstract Boolean drive(DcMotor[] Motors, double[] speeds);

    //Drive Settings
    public abstract void driveScale(double scale);

    public abstract void enableDrive(boolean onoff);

    public abstract void exponentialDriving(boolean onoff);

    public abstract void driveRange(double MinDriveValue, double MaxDriveValue);
}


