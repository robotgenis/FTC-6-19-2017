package com.pineapplerobotics.velocityvortex.auto.AutoS5;

import com.pineapplerobotics.velocityvortex.Stage2.AutoS2;
import com.pineapplerobotics.velocityvortex.javacodeclasses.resources.enums.PublicEnums;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by krisu on 11/10/2016.
 */
@Autonomous(name = "Blue AutoS5", group = "PineappleRobotics")

public class BlueAutoS5 extends AutoS2 {
    public BlueAutoS5(){super(PublicEnums.AllianceColor.BLUE, PublicEnums.wait.S5);}
}
