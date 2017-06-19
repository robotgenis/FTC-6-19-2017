package com.pineapplerobotics.velocityvortex.auto.Auto;

import com.pineapplerobotics.velocityvortex.Stage2.AutoS2;
import com.pineapplerobotics.velocityvortex.javacodeclasses.resources.enums.PublicEnums;
import com.pineapplerobotics.velocityvortex.javacodeclasses.resources.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by krisu on 11/10/2016.
 */
@Autonomous(name = "Red Auto", group = "PineappleRobotics")
public class RedAuto extends AutoS2 {
    public RedAuto(){super(PublicEnums.AllianceColor.RED, PublicEnums.wait.S0);}
}
