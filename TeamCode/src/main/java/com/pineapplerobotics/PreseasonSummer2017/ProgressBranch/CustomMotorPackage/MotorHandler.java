package com.pineapplerobotics.PreseasonSummer2017.ProgressBranch.CustomMotorPackage;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

/**
 * Created by Brandon on 6/19/2017.
 */

public class MotorHandler {

    HardwareMap hm;
    HashMap<String, CustomMotor> Motors = new HashMap<String, CustomMotor>();;

    public MotorHandler(HardwareMap hardwareMap){
        hm = hardwareMap;
    }

    public CustomMotor newMotor(String name){
        Motors.put(name, new CustomMotor(hm, name));
        return getMotor(name);
    }

    public CustomMotor getMotor(String name){
        if(!Motors.containsKey(name)){
            return null;
        }else{
            return Motors.get(name);
        }
    }



}
