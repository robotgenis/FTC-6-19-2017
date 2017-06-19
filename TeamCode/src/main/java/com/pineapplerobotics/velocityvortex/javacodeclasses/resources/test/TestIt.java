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
package com.pineapplerobotics.velocityvortex.javacodeclasses.resources.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the RobotHardware Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TestIt", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class TestIt extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // private DcMotor leftMotor = null;
    // private DcMotor rightMotor = null;

    ColorSensor beacon = new ColorSensor() {
        @Override
        public int red() {
            return 0;
        }

        @Override
        public int green() {
            return 0;
        }

        @Override
        public int blue() {
            return 0;
        }

        @Override
        public int alpha() {
            return 0;
        }

        @Override
        public int argb() {
            return 0;
        }

        @Override
        public void enableLed(boolean enable) {

        }

        @Override
        public void setI2cAddress(I2cAddr newAddress) {

        }

        @Override
        public I2cAddr getI2cAddress() {
            return null;
        }

        @Override
        public Manufacturer getManufacturer() {
            return null;
        }

        @Override
        public String getDeviceName() {
            return null;
        }

        @Override
        public String getConnectionInfo() {
            return null;
        }

        @Override
        public int getVersion() {
            return 0;
        }

        @Override
        public void resetDeviceConfigurationForOpMode() {

        }

        @Override
        public void close() {

        }
    };
    ColorSensor line = new ColorSensor() {
        @Override
        public int red() {
            return 0;
        }

        @Override
        public int green() {
            return 0;
        }

        @Override
        public int blue() {
            return 0;
        }

        @Override
        public int alpha() {
            return 0;
        }

        @Override
        public int argb() {
            return 0;
        }

        @Override
        public void enableLed(boolean enable) {

        }

        @Override
        public void setI2cAddress(I2cAddr newAddress) {

        }

        @Override
        public I2cAddr getI2cAddress() {
            return null;
        }

        @Override
        public Manufacturer getManufacturer() {
            return null;
        }

        @Override
        public String getDeviceName() {
            return null;
        }

        @Override
        public String getConnectionInfo() {
            return null;
        }

        @Override
        public int getVersion() {
            return 0;
        }

        @Override
        public void resetDeviceConfigurationForOpMode() {

        }

        @Override
        public void close() {

        }
    };

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        beacon = hardwareMap.colorSensor.get("beaconCS");

        line = hardwareMap.colorSensor.get("LineCS");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        beacon.enableLed(true);
        line.enableLed(true);

    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Beacon Color Blue:",beacon.blue());
        telemetry.addData("Beacon Color Red:",beacon.red());
        telemetry.addData("Beacon Color Green:",beacon.green());
        telemetry.addData("Line Color Blue:",line.blue());
        telemetry.addData("Line Color Red:",line.red());
        telemetry.addData("Line Color Green:",line.green());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
