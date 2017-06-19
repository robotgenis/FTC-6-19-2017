package com.pineapplerobotics.velocityvortex.auto.AutoS3;

import com.pineapplerobotics.velocityvortex.Stage2.AutoS2;
import com.pineapplerobotics.velocityvortex.javacodeclasses.resources.enums.PublicEnums;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by krisu on 11/10/2016.
 */
@Autonomous(name = "Blue AutoS3", group = "PineappleRobotics")
public class BlueAutoS3 extends AutoS2 {
    public BlueAutoS3(){super(PublicEnums.AllianceColor.BLUE, PublicEnums.wait.S3);}
}
