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
package com.pineapplerobotics.velocityvortex.javacodeclasses.resources;

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

public class Auto extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot;
    AllianceColor allianceColor;

    public Auto(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new RobotHardware(this);
        robot.initRobot();


        // Wait for the game to start (driver presses PLAY)
        this.waitForStart();

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
//                    robot.directionChange(PublicEnums.Direction.N);
            robot.driveToLine(45, 0.5, PublicEnums.Gyro.YES);
            robot.directionChange(PublicEnums.Direction.S);
            while (!robot.isThereBeacon() && opModeIsActive()) {
                robot.advancedLineFollow();
            }

            robot.leftBackMotor.setPower(.05);
            robot.leftFrontMotor.setPower(.05);
            robot.rightBackMotor.setPower(-.05);
            robot.rightFrontMotor.setPower(-.05);
            while (opModeIsActive() && !robot.t.isPressed()) {
            }
            robot.motorStop();
            robot.beaconBlue();
            robot.directionChange(PublicEnums.Direction.N);
            robot.drive(135, 0.5, 2000);
            robot.driveToLine(90, 0.25, PublicEnums.Gyro.NO);
            robot.directionChange(PublicEnums.Direction.S);
            while (!robot.isThereBeacon() && opModeIsActive()) {
                robot.advancedLineFollow();
            }
            robot.leftBackMotor.setPower(.05);
            robot.leftFrontMotor.setPower(.05);
            robot.rightBackMotor.setPower(-.05);
            robot.rightFrontMotor.setPower(-.05);
            while (opModeIsActive() && !robot.t.isPressed()) {
            }
            robot.motorStop();
            robot.beaconBlue();
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
            robot.directionChange(PublicEnums.Direction.N);
            robot.driveToLine(315, 0.5, PublicEnums.Gyro.YES);
            robot.directionChange(PublicEnums.Direction.S);
            while (!robot.isThereBeacon() && opModeIsActive()) {
                robot.advancedLineFollow();
            }

            robot.leftBackMotor.setPower(.05);
            robot.leftFrontMotor.setPower(.05);
            robot.rightBackMotor.setPower(-.05);
            robot.rightFrontMotor.setPower(-.05);
            while (opModeIsActive() && !robot.t.isPressed()) {
            }
            robot.motorStop();
            robot.beaconRed();
            robot.directionChange(PublicEnums.Direction.N);
            robot.drive(225, 0.5, 1000);
            robot.driveToLine(270, 0.25, PublicEnums.Gyro.NO);
            robot.directionChange(PublicEnums.Direction.S);
            while (!robot.isThereBeacon() && opModeIsActive()) {
                robot.advancedLineFollow();
            }
            robot.leftBackMotor.setPower(.05);
            robot.leftFrontMotor.setPower(.05);
            robot.rightBackMotor.setPower(-.05);
            robot.rightFrontMotor.setPower(-.05);
            while (opModeIsActive() && !robot.t.isPressed()) {
            }
            robot.motorStop();
            robot.beaconRed();


        }
    }

}
