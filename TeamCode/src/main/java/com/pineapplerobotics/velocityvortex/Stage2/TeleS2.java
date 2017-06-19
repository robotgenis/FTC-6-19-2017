package com.pineapplerobotics.velocityvortex.Stage2;


import com.pineapplerobotics.velocityvortex.javacodeclasses.resources.RobotVals;
import com.pineapplerobotics.velocityvortex.javacodeclasses.resources.enums.PublicEnums;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleS2", group = "Linear Opmode")
// @Autonomous(...) is the other common choice

public class TeleS2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    RobotHardwareS2 robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardwareS2(this);
        robot.initRobot();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            telemetry.addData("Status", "Run : " + runtime.toString());
            telemetry.addData("RUS", robot.rightBeaconUS);
            telemetry.addData("LUS", robot.leftBeaconUS);
            telemetry.addData("Gamepad1X:", gamepad1.left_stick_x);
            telemetry.addData("Gamepad1Y:", gamepad1.left_stick_y);
            telemetry.addData("Gamepad2 RBump:", gamepad2.right_bumper);
            telemetry.addData("Gamepad2 LBump:", gamepad2.left_bumper);
            telemetry.addData("Gamepad2 RTrig:", gamepad2.right_trigger);
            telemetry.addData("Gamepad2 LTrig:", gamepad2.right_trigger);

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

            Drive();

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

            if (gamepad1.x) {
                robot.lock.setPosition(0.8);
            } else {
                robot.lock.setPosition(0.3);
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
            //                                                    //
            ////////////////////////////////////////////////////////

            if (gamepad2.a) {
                robot.leftShooter.setPower(RobotVals.SHOOTER_SPEED);
                robot.rightShooter.setPower(RobotVals.SHOOTER_SPEED);
            }
            if (gamepad2.y) {
                robot.leftShooter.setPower(0);
                robot.rightShooter.setPower(0);
            }

            if (gamepad2.b) {
                robot.shoot.setPosition(.4);
            } else {
                robot.shoot.setPosition(.8);
            }

            if (gamepad2.dpad_left || gamepad2.dpad_up) {
                robot.beaconLeft.setPosition(1);
            } else {
                robot.beaconLeft.setPosition(0);
            }

            if (gamepad2.dpad_right || gamepad2.dpad_up) {
                robot.beaconRight.setPosition(0);
            } else {
                robot.beaconRight.setPosition(.95);
            }
            if (gamepad2.right_trigger > gamepad2.left_trigger) {
                robot.balllifter.setPower(gamepad2.right_trigger);
            } else {
                robot.balllifter.setPower(-gamepad2.left_trigger);
            }
            if (gamepad2.right_bumper) {
                robot.lift.setPower(1);
            } else if (gamepad2.left_bumper) {
                robot.lift.setPower(-1);
            } else {
                robot.lift.setPower(0);
            }

        }
    }

    public void Drive() {
        if (gamepad1.left_bumper) {
            double lfpower = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double rfpower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double lbpower = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            double rbpower = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            lfpower = Range.clip(lfpower, -1, 1) / 4;
            rfpower = Range.clip(rfpower, -1, 1) / 4;
            lbpower = Range.clip(lbpower, -1, 1) / 4;
            rbpower = Range.clip(rbpower, -1, 1) / 4;
            robot.leftFrontMotor.setPower(lfpower);
            robot.rightFrontMotor.setPower(rfpower);
            robot.leftBackMotor.setPower(lbpower);
            robot.rightBackMotor.setPower(rbpower);
        } else if (gamepad1.right_bumper) {
            double lfpower = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double rfpower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double lbpower = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            double rbpower = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            lfpower = Range.clip(lfpower, -1, 1);
            rfpower = Range.clip(rfpower, -1, 1);
            lbpower = Range.clip(lbpower, -1, 1);
            rbpower = Range.clip(rbpower, -1, 1);
            robot.leftFrontMotor.setPower(lfpower);
            robot.rightFrontMotor.setPower(rfpower);
            robot.leftBackMotor.setPower(lbpower);
            robot.rightBackMotor.setPower(rbpower);
        } else {
            double lfpower = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double rfpower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double lbpower = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            double rbpower = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            lfpower = Range.clip(lfpower, -1, 1) / 2;
            rfpower = Range.clip(rfpower, -1, 1) / 2;
            lbpower = Range.clip(lbpower, -1, 1) / 2;
            rbpower = Range.clip(rbpower, -1, 1) / 2;
            robot.leftFrontMotor.setPower(lfpower);
            robot.rightFrontMotor.setPower(rfpower);
            robot.leftBackMotor.setPower(lbpower);
            robot.rightBackMotor.setPower(rbpower);
        }

    }


}
