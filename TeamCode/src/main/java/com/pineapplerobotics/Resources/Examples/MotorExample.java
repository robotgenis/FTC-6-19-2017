package com.pineapplerobotics.Resources.Examples;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Brandon on 3/31/2017.
 */

@TeleOp(name = "Tele", group = "Linear Opmode")
@Disabled


public class MotorExample extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //here is will the DcMotor initialization Goes
    //Motor name goes where "ExampleMotor" is
    DcMotor ExampleMotor = null;
    //you set it to null but later will be changed

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //This is where the motor is connected to the name of the motor that is set on the phone
        //Where name is, is the name of the motor on the phone
        ExampleMotor = hardwareMap.dcMotor.get("Name");

        ExampleMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run : " + runtime.toString());
            telemetry.update();

            //This will get the number from the gamepad and set it to the variable
            double power = gamepad1.left_stick_x;

            //This sets the vvariabel to the motor
            ExampleMotor.setPower(power);

        }
    }

}

