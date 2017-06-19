package com.pineapplerobotics.velocityvortex.auto.Auto;

import com.pineapplerobotics.velocityvortex.Stage2.AutoS2;
import com.pineapplerobotics.velocityvortex.javacodeclasses.resources.enums.PublicEnums;
import com.pineapplerobotics.velocityvortex.javacodeclasses.resources.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by krisu on 11/10/2016.
 */
@Autonomous(name = "Blue Auto", group = "PineappleRobotics")
public class BlueAuto extends AutoS2 {
    public BlueAuto(){super(PublicEnums.AllianceColor.BLUE, PublicEnums.wait.S0);}
}
