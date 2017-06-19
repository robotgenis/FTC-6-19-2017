package com.pineapplerobotics.PreseasonSummer2017.ProgressBranch.CustomDrive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Brandon on 3/31/2017.
 */

public class OmniDrive extends RobotDrive {


    @Override
    public String driveType() {
        String type = "OmniDrive";
        return type;
    }

    @Override
    public Boolean gamepadDrive(DcMotor[] Motors, Gamepad gamepad) {
        if (Motors.length == 4) {

            return true;
        } else {
            return false;
        }
    }

    @Override
    public Boolean drive(DcMotor[] Motors, double[] speeds) {
        return null;
    }

    @Override
    public void driveScale(double scale) {

    }

    @Override
    public void enableDrive(boolean onoff) {

    }

    @Override
    public void exponentialDriving(boolean onoff) {

    }

    @Override
    public void driveRange(double MinDriveValue, double MaxDriveValue) {

    }

    @Override
    public String driveError() {
        return null;
    }

    @Override
    public double getDriveScale() {
        return 0;
    }

    @Override
    public boolean isDriveEnabled() {
        return false;
    }

    @Override
    public boolean isExponentialDrive() {
        return false;
    }

    @Override
    public double getMaxDriveValue() {
        return 0;
    }

    @Override
    public double getMinDriveValue() {
        return 0;
    }
}

