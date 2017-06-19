
package com.pineapplerobotics.PreseasonSummer2017.ProgressBranch.CustomMotorPackage;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Brandon on 3/31/2017.
 */

@TeleOp(name = "BlankLinearOpMode", group = "Linear Opmode")
@Disabled


public class CustomMotorTest extends com.qualcomm.robotcore.eventloop.opmode.LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    MotorHandler motors = new MotorHandler(hardwareMap);

    CustomMotor testMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        testMotor = motors.newMotor("motor");

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run : " + runtime.toString());
            telemetry.update();



        }
    }

}

