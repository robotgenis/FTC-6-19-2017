package com.pineapplerobotics.PreseasonSummer2017.ProgressBranch.CustomDrive;

import com.pineapplerobotics.PreseasonSummer2017.ProgressBranch.CustomDrive.RobotDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Brandon on 3/31/2017.
 */

public class TankDrive extends RobotDrive {
    boolean exponentialDrive = true;
    boolean driveEnabled = true;
    double driveMultiplier = 1;
    double MaxValue = 1;
    double MinValue = -1;
    String lastError = "No Error Message Given!";

    //Drive Settings
    @Override
    public void exponentialDriving(boolean onoff) {
        exponentialDrive = onoff;
    }

    @Override
    public void driveRange(double MinDriveValue, double MaxDriveValue) {
        MinValue = MinDriveValue;
        MaxValue = MaxDriveValue;
    }

    @Override
    public void driveScale(double scale) {
        driveMultiplier = scale;
    }

    @Override
    public void enableDrive(boolean onoff) {
        driveEnabled = onoff;
    }

    //Main Drive Function
    @Override
    public Boolean gamepadDrive(DcMotor[] Motors, Gamepad gamepad) {
        if (driveEnabled) {
            if (MinValue > MaxValue) {
                lastError = "Minimum Value is greater than Maximum value!";
                return false;
            } else {
                if (Motors.length == 2) {
                    double leftInput = gamepad.left_stick_y;
                    double rightInput = gamepad.right_stick_y;
                    Motors[0].setPower(motorPower(leftInput));
                    Motors[1].setPower(motorPower(rightInput));
                    return true;
                } else if (Motors.length == 4) {
                    double leftInput = gamepad.left_stick_y;
                    double rightInput = gamepad.right_stick_y;
                    Motors[0].setPower(motorPower(leftInput));
                    Motors[1].setPower(motorPower(rightInput));
                    Motors[2].setPower(motorPower(leftInput));
                    Motors[3].setPower(motorPower(rightInput));
                    return true;
                } else {
                    lastError = "Not the correct amount of motors!";
                    return false;
                }
            }
        } else {
            lastError = "Drive is not Enabled!";
            return false;
        }
    }

    @Override
    public Boolean drive(DcMotor[] Motors, double[] speeds) {
        if (driveEnabled) {
            if (MinValue > MaxValue) {
                lastError = "Minimum Value is greater than Maximum value!";
                return false;
            } else {
                if (Motors.length == 2 && speeds.length == 2) {
                    Motors[0].setPower(customPower(speeds[0], false, false));
                    Motors[1].setPower(customPower(speeds[1], false, false));
                    return true;
                } else if (Motors.length == 4 && speeds.length == 4) {
                    Motors[0].setPower(customPower(speeds[0], false, false));
                    Motors[1].setPower(customPower(speeds[1], false, false));
                    Motors[2].setPower(customPower(speeds[2], false, false));
                    Motors[3].setPower(customPower(speeds[3], false, false));
                    return true;
                } else {
                    lastError = "Not the correct amount of motors!";
                    return false;
                }
            }
        } else {
            lastError = "Drive is not Enabled!";
            return false;
        }
    }

    //Calculate Motor Ouputs
    public double motorPower(double input) {
        double output;
        if (exponentialDrive) {
            if (input < 0.0) {
                output = input * input;
                output = -output;
            } else {
                output = input * input;
            }
        } else {
            output = input;
        }
        output = output * driveMultiplier;
        output = Range.clip(output, MinValue, MaxValue);
        return output;
    }

    //Calculate Custom Motor Ouputs
    public double customPower(double input, boolean doExponentialDrive, boolean doScale) {
        double output;
        if (doExponentialDrive) {
            if (input < 0.0) {
                output = input * input;
                output = -output;
            } else {
                output = input * input;
            }
        } else {
            output = input;
        }
        if (doScale) {
            output = output * driveMultiplier;
        }
        output = Range.clip(output, -1, 1);
        return output;
    }

    //Gets Last Error
    @Override
    public String driveError() {
        return lastError;
    }

    //Get Information
    @Override
    public String driveType() {
        String type = "TankDrive";
        return type;
    }

    @Override
    public double getDriveScale() {
        return driveMultiplier;
    }

    @Override
    public boolean isDriveEnabled() {
        return driveEnabled;
    }

    @Override
    public boolean isExponentialDrive() {
        return exponentialDrive;
    }

    @Override
    public double getMaxDriveValue() {
        return MaxValue;
    }

    @Override
    public double getMinDriveValue() {
        return MinValue;
    }
}
