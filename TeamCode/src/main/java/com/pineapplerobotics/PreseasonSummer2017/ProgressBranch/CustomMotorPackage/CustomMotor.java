package com.pineapplerobotics.PreseasonSummer2017.ProgressBranch.CustomMotorPackage;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Brandon on 6/19/2017.
 */

public class CustomMotor{

    public double maxPower = 1;
    public double minPower = -1;

    public double defaultPower = 0;

    public double scaleBy = 1;

    public boolean exponetional = false;

    public boolean doDeadArea = false;

    double[] deadAreaArray = {};

    DcMotor motor;
    public String motorName;
    HardwareMap hm;

    public CustomMotor(HardwareMap hardwareMap, String name){
        hm = hardwareMap;
    }

    public void setScale(double scale, boolean exp){
        scaleBy = scale;
        exponetional = exp;
    }

    public void mapMotor(){
        motor = hm.dcMotor.get(motorName);
    }

    ///////////////////////
    //Set Power Functions//
    ///////////////////////

    public double setPower(double power){
        power = clip(power);
        motor.setPower(power);
        return power;
    }

    public double update(double power){
        return setPower(power);
    }

    public double update(boolean on){
        if(on) return setPower(maxPower);
        else return setPower(defaultPower);
    }

    public double update(boolean on, boolean off){
        if(on) return setPower(maxPower);
        else if(off) return setPower(minPower);
        else return setPower(defaultPower);
    }

    /////////////////////
    //Private Functions//
    /////////////////////

    private double fixValue(double input){
        input = scale(input);
        input = deadArea(input);
        input = clip(input);
        return input;
    }

    private double deadArea(double input){
        return input;
    }

    private double scale(double input){
        input = input*scaleBy;
        if(exponetional) input = input*input;
        return input;
    }

    private double clip(double input){
        input = Range.clip(input, minPower, maxPower);
        input = Range.clip(input, -1, 1);
        return input;
    }

}
