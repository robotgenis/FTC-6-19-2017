package com.pineapplerobotics.Resources.Examples;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Brandon on 3/31/2017.
 */

@TeleOp(name = "Tele", group = "Linear Opmode")
@Disabled


public class ServoExample extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //here is will the Servo initialization Goes
    //Motor name goes where "ExampleServo" is
    Servo ExampleServo = null;
    //you set it to null but later will be changed

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //This is where the servo is connected to the name of the servo that is set on the phone
        //Where name is, is the name of the servo on the phone
        ExampleServo = hardwareMap.servo.get("name");

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run : " + runtime.toString());
            telemetry.update();

            //This will get the number from the gamepad and set it to the variable
            double position = gamepad1.left_stick_x;

            //This sets the variabel to the Servo
            ExampleServo.setPosition(position);

        }
    }

}

