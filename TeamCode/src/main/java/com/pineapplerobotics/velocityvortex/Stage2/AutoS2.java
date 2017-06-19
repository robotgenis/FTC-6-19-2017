package com.pineapplerobotics.velocityvortex.Stage2;
/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

import com.pineapplerobotics.velocityvortex.javacodeclasses.resources.RobotHardware;
import com.pineapplerobotics.velocityvortex.javacodeclasses.resources.enums.PublicEnums;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.pineapplerobotics.velocityvortex.javacodeclasses.resources.enums.PublicEnums.AllianceColor.BLUE;
import static com.pineapplerobotics.velocityvortex.javacodeclasses.resources.enums.PublicEnums.AllianceColor.RED;


import static com.pineapplerobotics.velocityvortex.javacodeclasses.resources.enums.PublicEnums.AllianceColor;
import static com.pineapplerobotics.velocityvortex.javacodeclasses.resources.enums.PublicEnums.AllianceColor.BLUE;
import static com.pineapplerobotics.velocityvortex.javacodeclasses.resources.enums.PublicEnums.AllianceColor.RED;

import com.pineapplerobotics.velocityvortex.javacodeclasses.resources.enums.PublicEnums;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the RobotHardware Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Auto", group = "Linear Opmode")
@Disabled

public class AutoS2 extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardwareS2 robot;
    PublicEnums.AllianceColor allianceColor;
    PublicEnums.wait wait;

    public AutoS2(PublicEnums.AllianceColor allianceColor, PublicEnums.wait wait) {
        this.allianceColor = allianceColor;
        this.wait = wait;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new RobotHardwareS2(this);
        robot.initRobot();


        // Wait for the game to start (driver presses PLAY)
        this.waitForStart();
        robot.directionChange(PublicEnums.Direction.E);

        switch (wait) {
            case S0:

                break;
            case S3:
                Thread.sleep(3000);
                break;
            case S5:
                Thread.sleep(5000);
                break;
            case S7:
                Thread.sleep(7000);
                break;
            case S10:
                Thread.sleep(10000);
                break;
        }
        ////////////////////////////////////////////////////////
        //                       BLUE                         //
        //                                                    //
        //                                                    //
        //                                                    //
        //                                                    //
        //                                                    //
        //                                                    //
        //                                                    //
        ////////////////////////////////////////////////////////
        if (allianceColor == BLUE) {
            switch (wait) {
                case S0:
                    robot.beaconServoReset();
                    robot.directionChange(PublicEnums.Direction.E);
                    robot.driveToLine(45, 1, PublicEnums.Gyro.YES);
                    robot.drive(45, 0.5, 100);
                    robot.alignWithWallUS();
                    robot.driveWithUS(0, 0.5, 13);
                    robot.alignWithWallUS();
                    robot.driveToLine(270, 0.4, PublicEnums.Gyro.NO);
                    robot.drive(270, 0.2, 100);
                    robot.driveWithUS(0, 0.4, 9);
                    robot.beaconBlue();
                    robot.alignWithWallUS();
                    robot.driveToUSPost(180, 0.5, 13, PublicEnums.USside.LEFT);
                    robot.shoot(2);
                    robot.drive(125, 1, 1000);
                    robot.driveToLine(90, 1, PublicEnums.Gyro.NO);
                    robot.drive(45, 0.5, 200);
                    robot.alignWithWallUS();
                    robot.driveWithUS(0, 0.5, 13);
                    robot.alignWithWallUS();
                    robot.driveToLine(270, 0.4, PublicEnums.Gyro.NO);
                    robot.drive(270, 0.2, 100);
                    robot.driveWithUS(0, 0.4, 9);
                    robot.beaconBlue();
                    break;
                case S3:
                    robot.beaconServoReset();
                    robot.directionChange(PublicEnums.Direction.E);
                    robot.driveToLine(45, 1, PublicEnums.Gyro.YES);
                    robot.drive(45, 0.5, 100);
                    robot.alignWithWallUS();
                    robot.driveWithUS(0, 0.5, 13);
                    robot.alignWithWallUS();
                    robot.driveToLine(270, 0.4, PublicEnums.Gyro.NO);
                    robot.drive(270, 0.2, 100);
                    robot.driveWithUS(0, 0.4, 9);
                    robot.beaconBlue();
                    robot.alignWithWallUS();
                    robot.driveToUSPost(180, 0.5, 13, PublicEnums.USside.LEFT);
                    robot.shoot(2);
                    robot.drive(125, 1, 1000);
                    robot.driveToLine(90, 1, PublicEnums.Gyro.NO);
                    robot.drive(45, 0.5, 200);
                    robot.alignWithWallUS();
                    robot.driveWithUS(0, 0.5, 13);
                    robot.alignWithWallUS();
                    robot.driveToLine(270, 0.4, PublicEnums.Gyro.NO);
                    robot.drive(270, 0.2, 100);
                    robot.driveWithUS(0, 0.4, 9);
                    robot.beaconBlue();
                    break;
                case S5:
                    robot.beaconServoReset();
                    robot.directionChange(PublicEnums.Direction.E);
                    robot.driveToLine(45, 1, PublicEnums.Gyro.YES);
                    robot.drive(45, 0.5, 100);
                    robot.alignWithWallUS();
                    robot.driveWithUS(0, 0.5, 13);
                    robot.alignWithWallUS();
                    robot.driveToLine(270, 0.4, PublicEnums.Gyro.NO);
                    robot.drive(270, 0.2, 100);
                    robot.driveWithUS(0, 0.4, 9);
                    robot.beaconBlue();
                    robot.alignWithWallUS();
                    robot.driveToUSPost(180, 0.5, 13, PublicEnums.USside.LEFT);
                    robot.shoot(2);
                    break;
                case S7:
                    robot.beaconServoReset();
                    robot.directionChange(PublicEnums.Direction.E);
                    robot.driveToLine(45, 1, PublicEnums.Gyro.YES);
                    robot.drive(45, 0.5, 100);
                    robot.alignWithWallUS();
                    robot.driveWithUS(0, 0.5, 13);
                    robot.alignWithWallUS();
                    robot.driveToLine(270, 0.4, PublicEnums.Gyro.NO);
                    robot.drive(270, 0.2, 100);
                    robot.driveWithUS(0, 0.4, 9);
                    robot.beaconBlue();
                    robot.alignWithWallUS();
                    robot.driveToUSPost(180, 0.5, 13, PublicEnums.USside.LEFT);
                    robot.shoot(2);
                    break;
                case S10:
                    robot.beaconServoReset();
                    robot.directionChange(PublicEnums.Direction.E);
                    robot.driveToLine(45, 1, PublicEnums.Gyro.YES);
                    robot.drive(45, 0.5, 100);
                    robot.alignWithWallUS();
                    robot.driveWithUS(0, 0.5, 13);
                    robot.alignWithWallUS();
                    robot.driveToLine(270, 0.4, PublicEnums.Gyro.NO);
                    robot.drive(270, 0.2, 100);
                    robot.driveWithUS(0, 0.4, 9);
                    robot.beaconBlue();

                    break;
            }
        }

        ////////////////////////////////////////////////////////
        //                        RED                         //
        //                                                    //
        //                                                    //
        //                                                    //
        //                                                    //
        //                                                    //
        //                                                    //
        //                                                    //
        ////////////////////////////////////////////////////////


        else if (allianceColor == RED) {
            switch (wait) {
                case S0:
                    robot.beaconServoReset();
                    robot.directionChange(PublicEnums.Direction.E);
                    robot.driveToLine(315, 1, PublicEnums.Gyro.YES);
                    robot.drive(315, 0.5, 100);
                    robot.alignWithWallUS();
                    robot.driveWithUS(0, 0.5, 13);
                    robot.alignWithWallUS();
                    robot.driveToLine(90, 0.4, PublicEnums.Gyro.NO);
                    robot.drive(90, 0.2, 100);
                    robot.driveWithUS(0, 0.4, 9);
                    robot.beaconRed();
                    robot.alignWithWallUS();
                    robot.driveToUSPost(180, 0.5, 13, PublicEnums.USside.LEFT);
                    robot.shoot(2);
                    robot.drive(235, 1, 1000);
                    robot.driveToLine(270, 1, PublicEnums.Gyro.NO);
                    robot.drive(315, 0.5, 200);
                    robot.alignWithWallUS();
                    robot.driveWithUS(0, 0.5, 13);
                    robot.alignWithWallUS();
                    robot.driveToLine(90, 0.4, PublicEnums.Gyro.NO);
                    robot.drive(90, 0.2, 100);
                    robot.driveWithUS(0, 0.4, 9);
                    robot.beaconRed();
                    break;
                case S3:
                    robot.beaconServoReset();
                    robot.directionChange(PublicEnums.Direction.E);
                    robot.driveToLine(315, 1, PublicEnums.Gyro.YES);
                    robot.drive(315, 0.5, 100);
                    robot.alignWithWallUS();
                    robot.driveWithUS(0, 0.5, 13);
                    robot.alignWithWallUS();
                    robot.driveToLine(90, 0.4, PublicEnums.Gyro.NO);
                    robot.drive(90, 0.2, 100);
                    robot.driveWithUS(0, 0.4, 9);
                    robot.beaconRed();
                    robot.alignWithWallUS();
                    robot.driveToUSPost(180, 0.5, 13, PublicEnums.USside.LEFT);
                    robot.shoot(2);
                    robot.drive(235, 1, 1000);
                    robot.driveToLine(270, 1, PublicEnums.Gyro.NO);
                    robot.drive(315, 0.5, 200);
                    robot.alignWithWallUS();
                    robot.driveWithUS(0, 0.5, 13);
                    robot.alignWithWallUS();
                    robot.driveToLine(90, 0.4, PublicEnums.Gyro.NO);
                    robot.drive(90, 0.2, 100);
                    robot.driveWithUS(0, 0.4, 9);
                    robot.beaconRed();
                    break;
                case S5:
                    robot.beaconServoReset();
                    robot.directionChange(PublicEnums.Direction.E);
                    robot.driveToLine(315, 1, PublicEnums.Gyro.YES);
                    robot.drive(315, 0.5, 100);
                    robot.alignWithWallUS();
                    robot.driveWithUS(0, 0.5, 13);
                    robot.alignWithWallUS();
                    robot.driveToLine(90, 0.4, PublicEnums.Gyro.NO);
                    robot.drive(90, 0.2, 100);
                    robot.driveWithUS(0, 0.4, 9);
                    robot.beaconRed();
                    robot.alignWithWallUS();
                    robot.driveToUSPost(180, 0.5, 13, PublicEnums.USside.LEFT);
                    robot.shoot(2);
                    break;
                case S7:
                    robot.beaconServoReset();
                    robot.directionChange(PublicEnums.Direction.E);
                    robot.driveToLine(315, 1, PublicEnums.Gyro.YES);
                    robot.drive(315, 0.5, 100);
                    robot.alignWithWallUS();
                    robot.driveWithUS(0, 0.5, 13);
                    robot.alignWithWallUS();
                    robot.driveToLine(90, 0.4, PublicEnums.Gyro.NO);
                    robot.drive(90, 0.2, 100);
                    robot.driveWithUS(0, 0.4, 9);
                    robot.beaconRed();
                    robot.alignWithWallUS();
                    robot.driveToUSPost(180, 0.5, 13, PublicEnums.USside.LEFT);
                    robot.shoot(2);
                    break;
                case S10:
                    robot.beaconServoReset();
                    robot.directionChange(PublicEnums.Direction.E);
                    robot.driveToLine(315, 1, PublicEnums.Gyro.YES);
                    robot.drive(315, 0.5, 100);
                    robot.alignWithWallUS();
                    robot.driveWithUS(0, 0.5, 13);
                    robot.alignWithWallUS();
                    robot.driveToLine(90, 0.4, PublicEnums.Gyro.NO);
                    robot.drive(90, 0.2, 100);
                    robot.driveWithUS(0, 0.4, 9);
                    robot.beaconRed();
                    break;
            }
        }
    }
}