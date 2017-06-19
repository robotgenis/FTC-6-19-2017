package com.pineapplerobotics.Resources.Examples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Brandon on 4/2/2017.
 */

public class TeleOpExample extends com.qualcomm.robotcore.eventloop.opmode.LinearOpMode{

    DcMotor left_motor = null;
    DcMotor right_motor = null;

    Servo servo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        left_motor = hardwareMap.dcMotor.get("leftmotor");
        right_motor = hardwareMap.dcMotor.get("rightmotor");

        servo = hardwareMap.servo.get("armservo");

        waitForStart();
        while (opModeIsActive()){


            double left = gamepad1.left_stick_x;
            double right = gamepad1.right_stick_x;

            telemetry.addData("Left Motor", left);
            telemetry.addData("Right Motor", right);
            telemetry.update();

            left_motor.setPower(left); // -1 to 1
            right_motor.setPower(right);

            if (gamepad1.a){
                servo.setPosition(.8); // 0 to 1
            }else{
                servo.setPosition(0);
            }
        }

    }
}
