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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;

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

@TeleOp(name = "Shooter", group = "Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class Shooter extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    //Dc
    //
    // Motor flipperMotor = null;Coo
    CRServo lift = null;
Servo shoot = null;
    DcMotor leftShooter = null;
    DcMotor rightShooter = null;

    DcMotor motorRF = null;
    DcMotor motorRB = null;
    DcMotor motorLF = null;
    DcMotor motorLB = null;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to  names assigned during the robot configuration
         * step (using the FTC RobotHardware Controller app on the phone).
         */

        shoot = hardwareMap.servo.get("s");
        rightShooter = hardwareMap.dcMotor.get("s1");
        leftShooter = hardwareMap.dcMotor.get("s2");
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        lift = hardwareMap.crservo.get("cs");
        motorLF = hardwareMap.dcMotor.get("lf");
        motorRF = hardwareMap.dcMotor.get("rf");
        motorLB = hardwareMap.dcMotor.get("lb");
        motorRB = hardwareMap.dcMotor.get("rb");
        waitForStart();
        runtime.reset();

         shoot.setPosition(.8);

        while (opModeIsActive()) {



            telemetry.addData("Status", "Run : " + runtime.toString());
            telemetry.update();
            if(gamepad1.a) {
                leftShooter.setPower(0.25);
                rightShooter.setPower(0.25);
            }
            if (gamepad1.y){
                leftShooter.setPower(0);
                rightShooter.setPower(0);
            }
            if (gamepad1.b){
                shoot.setPosition(.4);
            }else{
                shoot.setPosition(.8);
            }
            lift.setPower(gamepad1.right_trigger);

        }
    }

}
