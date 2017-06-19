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

import com.pineapplerobotics.velocityvortex.javacodeclasses.resources.enums.PublicEnums;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name = "Tele", group = "Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class Tele extends LinearOpMode {
    RobotHardware robot;
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    //DcMotor flipperMotor = null;

    double speedvalvar = 0;
    double x = .7;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(this);
        robot.initRobot();
        robot.directionChange(PublicEnums.Direction.E);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to  names assigned during the robot configuration
         * step (using the FTC RobotHardware Controller app on the phone).
         */


        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            telemetry.addData("Status", "Run : " + runtime.toString());
            telemetry.update();
            ////////////////////////////////////////////////////////
            //                    GAMEPAD 1                       //
            //                                                    //
            //                                                    //
            //                                                    //
            //                                                    //
            //                                                    //
            //                                                    //
            //                                                    //
            ////////////////////////////////////////////////////////
            if (speedvalvar == 0) {
                if (robot.robotDirection == PublicEnums.Direction.W || robot.robotDirection == PublicEnums.Direction.E) {
                    robot.leftFrontMotor.setPower(Range.clip((gamepad1.left_stick_y + gamepad1.left_stick_x) / 2 - (gamepad1.right_stick_x) / 2, -1, 1));
                    robot.rightFrontMotor.setPower(Range.clip((gamepad1.left_stick_y - gamepad1.left_stick_x) / 2 - (gamepad1.right_stick_x) / 2, -1, 1));
                    robot.leftBackMotor.setPower(Range.clip((-gamepad1.left_stick_y + gamepad1.left_stick_x) / 2 - (gamepad1.right_stick_x) / 2, -1, 1));
                    robot.rightBackMotor.setPower(Range.clip((-gamepad1.left_stick_y - gamepad1.left_stick_x) / 2 - (gamepad1.right_stick_x) / 2, -1, 1));
                }
                if (robot.robotDirection == PublicEnums.Direction.N || robot.robotDirection == PublicEnums.Direction.S) {
                    robot.leftFrontMotor.setPower(Range.clip((gamepad1.left_stick_y + gamepad1.left_stick_x) / 2 - (gamepad1.right_stick_x) / 2, -1, 1));
                    robot.rightFrontMotor.setPower(Range.clip((gamepad1.left_stick_y - gamepad1.left_stick_x) / 2 - (gamepad1.right_stick_x) / 2, -1, 1));
                    robot.leftBackMotor.setPower(Range.clip((-gamepad1.left_stick_y + gamepad1.left_stick_x) / 2 - (gamepad1.right_stick_x) / 2, -1, 1));
                    robot.rightBackMotor.setPower(Range.clip((-gamepad1.left_stick_y - gamepad1.left_stick_x) / 2 - (gamepad1.right_stick_x) / 2, -1, 1));
                }
            }
            if (speedvalvar == 1) {
                if (robot.robotDirection == PublicEnums.Direction.W || robot.robotDirection == PublicEnums.Direction.E) {
                    robot.leftFrontMotor.setPower(Range.clip((gamepad1.left_stick_y + gamepad1.left_stick_x) / 2 - (gamepad1.right_stick_x) / 2, -1, 1)/3);
                    robot.rightFrontMotor.setPower(Range.clip((gamepad1.left_stick_y - gamepad1.left_stick_x) / 2 - (gamepad1.right_stick_x) / 2, -1, 1)/3);
                    robot.leftBackMotor.setPower(Range.clip((-gamepad1.left_stick_y + gamepad1.left_stick_x) / 2 - (gamepad1.right_stick_x) / 2, -1, 1)/3);
                    robot.rightBackMotor.setPower(Range.clip((-gamepad1.left_stick_y - gamepad1.left_stick_x) / 2 - (gamepad1.right_stick_x) / 2, -1, 1)/3);
                }
                if (robot.robotDirection == PublicEnums.Direction.N || robot.robotDirection == PublicEnums.Direction.S) {
                    robot.leftFrontMotor.setPower(Range.clip((gamepad1.left_stick_y + gamepad1.left_stick_x) / 2 - (gamepad1.right_stick_x) / 2, -1, 1)/3);
                    robot.rightFrontMotor.setPower(Range.clip((gamepad1.left_stick_y - gamepad1.left_stick_x) / 2 - (gamepad1.right_stick_x) / 2, -1, 1)/3);
                    robot.leftBackMotor.setPower(Range.clip((-gamepad1.left_stick_y + gamepad1.left_stick_x) / 2 - (gamepad1.right_stick_x) / 2, -1, 1)/3);
                    robot.rightBackMotor.setPower(Range.clip((-gamepad1.left_stick_y - gamepad1.left_stick_x) / 2 - (gamepad1.right_stick_x) / 2, -1, 1)/3);
                }
            }
            if (gamepad1.dpad_up) {
                robot.directionChange(PublicEnums.Direction.E);
            }
            if (gamepad1.dpad_left) {
                robot.directionChange(PublicEnums.Direction.S);
            }
            if (gamepad1.dpad_down) {
                robot.directionChange(PublicEnums.Direction.W);
            }
            if (gamepad1.dpad_right) {
                robot.directionChange(PublicEnums.Direction.N);
            }
            if (gamepad1.right_trigger >= .2) {
                speedvalvar = 0;
            } else if (gamepad1.left_trigger >= .2) {
                speedvalvar = 1;
            }

            ////////////////////////////////////////////////////////
            //                    GAMEPAD 2                       //
            //                                                    //
            //                                                    //
            //                                                    //
            //                                                    //
            //                                                    //
            //                                                    //
            //                                                    //
            ////////////////////////////////////////////////////////


            if (gamepad2.b) {
                robot.beaconRight.setPosition(RobotVals.BEACON_RIGHT_FORWARD);
            } else if (gamepad2.x) {
                robot.beaconLeft.setPosition(RobotVals.BEACON_LEFT_FORWARD);
            } else {
                robot.beaconServoReset();
            }
            if (gamepad2.dpad_up) {
                robot.leftLifter.setPower(1);
                robot.rightLifter.setPower(1);

            } else if (gamepad2.dpad_down) {
                robot.leftLifter.setPower(-0.25);
                robot.rightLifter.setPower(-0.25);

            } else {
                robot.rightLifter.setPower(0);
                robot.leftLifter.setPower(0);

            }
            if (gamepad2.right_bumper) {
                robot.lock.setPosition(0);
            } else if (gamepad2.left_bumper) {
                robot.lock.setPosition(1);
            }
//            if (gamepad2.dpad_up) {
//                robot.liftToTop();
//            } else if (gamepad2.dpad_down) {
//                robot.liftToBottem();
//            } else {
//                robot.rightLifter.setPower(0);
//                robot.leftLifter.setPower(0);
//            }


        }
    }

}
