package com.pineapplerobotics.velocityvortex.auto.AutoS10;

import com.pineapplerobotics.velocityvortex.Stage2.AutoS2;
import com.pineapplerobotics.velocityvortex.javacodeclasses.resources.enums.PublicEnums;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by krisu on 11/10/2016.
 */
@Autonomous(name = "Red AutoS10", group = "PineappleRobotics")
public class RedAutoS10 extends AutoS2 {
    public RedAutoS10(){super(PublicEnums.AllianceColor.RED, PublicEnums.wait.S10);}
}
