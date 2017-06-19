//This will auto generate and just tells the porgram where it is located
package com.pineapplerobotics.Resources.Examples;

//This is where anything will be imported tp the program
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Brandon on 3/31/2017.
 */

//This Allows the program to run on the phone
//Change the name to change the name of the program on the phone

@TeleOp(name = "Tele", group = "Linear Opmode")

//This Keeps it from showing without deleting the previous line
@Disabled

//This is where all the code is going to go
public class LinearOpModeStructure extends LinearOpMode {
    //This creates a timer that will later be used to tell how long the program is active
    private ElapsedTime runtime = new ElapsedTime();
    //All initialization Goes right Here

    //This is where the program is run
    @Override
    public void runOpMode() throws InterruptedException {
        //The telemetry is data that shows on the screen
        telemetry.addData("Status", "Initialized");
        //You update it to show it on the screen
        telemetry.update();
        //This is where the hardware setup goes

        //This is where the program starts and waits for you to click the button
        waitForStart();
        //This resets the timer created at the begining
        runtime.reset();
        //This loop is where the code is run, the loop keeps repeating the code untill the program is stopped but if you don't put a loop then it
        //will run each line of code one at a time-this might be used for autonomous
        while (opModeIsActive()) {
            //This shows how long the program is running
            telemetry.addData("Status", "Run : " + runtime.toString());
            telemetry.update();
            //You would add all code to move the robot or get inputs here
        }
    }

}

