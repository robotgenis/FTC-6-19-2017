package com.pineapplerobotics.velocityvortex.Stage2;

import com.pineapplerobotics.velocityvortex.javacodeclasses.resources.RobotVals;
import com.pineapplerobotics.velocityvortex.javacodeclasses.resources.enums.PublicEnums;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class RobotHardwareS2 {
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor rightBackMotor = null;

    public DcMotor rightShooter = null;
    public DcMotor leftShooter = null;

    public PublicEnums.Direction robotDirection = PublicEnums.Direction.N;

    public Servo beaconRight = null;
    public Servo beaconLeft = null;
    public Servo lock = null;
    public Servo shoot = null;
    public DcMotor lift = null;
    public DcMotor balllifter = null;

    public OpticalDistanceSensor lineODS = null;
    public GyroSensor robotGS = null;
    public ColorSensor beaconCS = null;


    public double matColorVal = 0;

    public int startDirection = 0;
    //public TouchSensor t = null;

    public UltrasonicSensor rightBeaconUS = null;
    public UltrasonicSensor leftBeaconUS = null;
    LinearOpMode opMode;

    public RobotHardwareS2(LinearOpMode opMode) {
        this.opMode = opMode;
        leftFrontMotor = opMode.hardwareMap.dcMotor.get("lf");
        rightFrontMotor = opMode.hardwareMap.dcMotor.get("rf");
        leftBackMotor = opMode.hardwareMap.dcMotor.get("lb");
        rightBackMotor = opMode.hardwareMap.dcMotor.get("rb");
        rightShooter = opMode.hardwareMap.dcMotor.get("rightShooter");
        leftShooter = opMode.hardwareMap.dcMotor.get("leftShooter");
        shoot = opMode.hardwareMap.servo.get("shoot");
        beaconRight = opMode.hardwareMap.servo.get("beaconR");
        beaconLeft = opMode.hardwareMap.servo.get("beaconL");
        lock = opMode.hardwareMap.servo.get("lock");
        lift = opMode.hardwareMap.dcMotor.get("lift");
        balllifter = opMode.hardwareMap.dcMotor.get("balllifter");
        lineODS = opMode.hardwareMap.opticalDistanceSensor.get("ODS");
        robotGS = opMode.hardwareMap.gyroSensor.get("GS");
        beaconCS = opMode.hardwareMap.colorSensor.get("CS");
        rightBeaconUS = opMode.hardwareMap.ultrasonicSensor.get("RUS");
        leftBeaconUS = opMode.hardwareMap.ultrasonicSensor.get("LUS");

    }

    public void initRobot() throws InterruptedException {
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightShooter.setPower(0);
        leftShooter.setPower(0);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
//        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
//        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        shoot.setPosition(0.8);
        //beaconServoReset();
        beaconCS.enableLed(false);
        lineODS.enableLed(true);

        matColorVal = lineODS.getLightDetected();
        //   robotGS.calibrate();

//        startDirection = robotGS.getHeading();
    }

    ////////////////////////////////////////////////////////
    //                   Drive Programs                   //
    //                                                    //
    //                                                    //
    //                                                    //
    //                                                    //
    //                                                    //
    //                                                    //
    //                                                    //
    ////////////////////////////////////////////////////////
    public void directionChange(PublicEnums.Direction direction) {
        robotDirection = direction;

        switch (direction) {
            case N:
                leftFrontMotor = opMode.hardwareMap.dcMotor.get("lf");
                rightFrontMotor = opMode.hardwareMap.dcMotor.get("rf");
                leftBackMotor = opMode.hardwareMap.dcMotor.get("lb");
                rightBackMotor = opMode.hardwareMap.dcMotor.get("rb");
                opMode.telemetry.addData("Robot Direction:", "N");
                break;
            case E:
                leftFrontMotor = opMode.hardwareMap.dcMotor.get("lb");
                rightFrontMotor = opMode.hardwareMap.dcMotor.get("lf");
                leftBackMotor = opMode.hardwareMap.dcMotor.get("rb");
                rightBackMotor = opMode.hardwareMap.dcMotor.get("rf");
                opMode.telemetry.addData("Robot Direction:", "E");

                break;
            case S:
                leftFrontMotor = opMode.hardwareMap.dcMotor.get("rb");
                rightFrontMotor = opMode.hardwareMap.dcMotor.get("lb");
                leftBackMotor = opMode.hardwareMap.dcMotor.get("rf");
                rightBackMotor = opMode.hardwareMap.dcMotor.get("lf");
                opMode.telemetry.addData("Robot Direction:", "S");

                break;
            case W:
                leftFrontMotor = opMode.hardwareMap.dcMotor.get("rf");
                rightFrontMotor = opMode.hardwareMap.dcMotor.get("rb");
                leftBackMotor = opMode.hardwareMap.dcMotor.get("lf");
                rightBackMotor = opMode.hardwareMap.dcMotor.get("lb");
                opMode.telemetry.addData("Robot Direction:", "W");

                break;
            default:
                leftFrontMotor = opMode.hardwareMap.dcMotor.get("lf");
                rightFrontMotor = opMode.hardwareMap.dcMotor.get("rf");
                leftBackMotor = opMode.hardwareMap.dcMotor.get("lb");
                rightBackMotor = opMode.hardwareMap.dcMotor.get("rb");
                opMode.telemetry.addData("Robot Direction:", "N");
                break;

        }
        opMode.telemetry.update();
    }
    public void gyroAlign() {
        double currentDirection;
        double targetDirection = startDirection;
        if (targetDirection <= 180 && targetDirection > 0) {
            currentDirection = robotGS.getHeading();
            if (currentDirection > targetDirection) {
                while (currentDirection > targetDirection) {
                    leftBackMotor.setPower(0.2);
                    leftFrontMotor.setPower(0.2);
                    rightBackMotor.setPower(0.2);
                    rightFrontMotor.setPower(0.2);
                    currentDirection = robotGS.getHeading();
                    opMode.telemetry.addData("Robot Heading", robotGS.getHeading());
                    opMode.telemetry.update();
                }
            } else if (currentDirection < targetDirection) {
                while (currentDirection < targetDirection) {
                    leftBackMotor.setPower(-0.2);
                    leftFrontMotor.setPower(-0.2);
                    rightBackMotor.setPower(-0.2);
                    rightFrontMotor.setPower(-0.2);
                    currentDirection = robotGS.getHeading();
                    opMode.telemetry.addData("Robot Heading", robotGS.getHeading());
                    opMode.telemetry.update();
                }
            }
        } else {
            currentDirection = robotGS.getHeading();
            if (currentDirection > targetDirection) {
                while (currentDirection < targetDirection) {
                    leftBackMotor.setPower(0.2);
                    leftFrontMotor.setPower(0.2);
                    rightBackMotor.setPower(0.2);
                    rightFrontMotor.setPower(0.2);
                    currentDirection = robotGS.getHeading();
                    opMode.telemetry.addData("Robot Heading", robotGS.getHeading());
                    opMode.telemetry.update();
                }
            } else if (currentDirection < targetDirection) {
                while (currentDirection > targetDirection) {
                    leftBackMotor.setPower(-0.2);
                    leftFrontMotor.setPower(-0.2);
                    rightBackMotor.setPower(-0.2);
                    rightFrontMotor.setPower(-0.2);
                    currentDirection = robotGS.getHeading();
                    opMode.telemetry.addData("Robot Heading", robotGS.getHeading());
                    opMode.telemetry.update();
                }
            }

        }
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }

    public double[] getXY(double degrees, double speed) {


        double[] XY = new double[2];
        double x = 0;
        double y = 0;

        x = (speed * Math.cos(degrees * Math.PI / 180F));
        y = (speed * Math.sin(degrees * Math.PI / 180F));
        x = Math.round(x * 1000.0) / 1000.0;
        y = Math.round(y * 1000.0) / 1000.0;
        XY[0] = x;
        XY[1] = y;
        return XY;
    }

    public void drive(double degrees, double speed, double time) {
        double x = getXY(degrees, speed)[0];
        double y = getXY(degrees, speed)[1];
        ElapsedTime runtime = new ElapsedTime();

        while (runtime.milliseconds() <= time && opMode.opModeIsActive()) {
            leftFrontMotor.setPower(((-x + y) / 2));
            leftBackMotor.setPower(((-x - y) / 2));
            rightFrontMotor.setPower(((x + y) / 2));
            rightBackMotor.setPower(((x - y) / 2));

        }
    }

    public void alignWithWallUS() {
        double startDistR = rightBeaconUS.getUltrasonicLevel();
        double startDistL = leftBeaconUS.getUltrasonicLevel();
        if (startDistL > startDistR) {
            while (startDistL != startDistR && opMode.opModeIsActive()) {
                opMode.telemetry.addData("Right US", startDistR);
                opMode.telemetry.addData("Left US", startDistL);
                opMode.telemetry.update();
                leftBackMotor.setPower(0.1);
                leftFrontMotor.setPower(0.1);
                rightBackMotor.setPower(0.1);
                rightFrontMotor.setPower(0.1);
                startDistR = rightBeaconUS.getUltrasonicLevel();
                startDistL = leftBeaconUS.getUltrasonicLevel();
            }
        } else if (startDistL < startDistR) {
            while (startDistL != startDistR && opMode.opModeIsActive()) {
                leftBackMotor.setPower(-0.1);
                leftFrontMotor.setPower(-0.1);
                rightBackMotor.setPower(-0.1);
                rightFrontMotor.setPower(-0.1);
                startDistR = rightBeaconUS.getUltrasonicLevel();
                startDistL = leftBeaconUS.getUltrasonicLevel();
            }
        }
    }

    public void motorStop() {
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }

    public void driveWithUS(double degrees, double speed, double target) {
        double x = getXY(degrees, speed)[0];
        double y = getXY(degrees, speed)[1];
        while (leftBeaconUS.getUltrasonicLevel() > target && rightBeaconUS.getUltrasonicLevel() > target && opMode.opModeIsActive()) {

            if (leftBeaconUS.getUltrasonicLevel() < rightBeaconUS.getUltrasonicLevel()) {
                leftFrontMotor.setPower(((-x + y) / 2));
                leftBackMotor.setPower(((-x - y) / 2));
                rightFrontMotor.setPower(((x + y) / 2) / 2);
                rightBackMotor.setPower(((x - y) / 2) / 2);
            } else if (leftBeaconUS.getUltrasonicLevel() > rightBeaconUS.getUltrasonicLevel()) {
                leftFrontMotor.setPower(((-x + y) / 2) / 2);
                leftBackMotor.setPower(((-x - y) / 2) / 2);
                rightFrontMotor.setPower(((x + y) / 2));
                rightBackMotor.setPower(((x - y) / 2));
            } else {
                leftFrontMotor.setPower(((-x + y) / 2));
                leftBackMotor.setPower(((-x - y) / 2));
                rightFrontMotor.setPower(((x + y) / 2));
                rightBackMotor.setPower(((x - y) / 2));
            }
            opMode.telemetry.addData("RightUS:", rightBeaconUS.getUltrasonicLevel());
            opMode.telemetry.addData("LeftUS:", leftBeaconUS.getUltrasonicLevel());
            opMode.telemetry.update();
        }
        motorStop();
    }

    public void driveToUSPost(double degrees, double speed, double target, PublicEnums.USside uSside) {
        double x = getXY(degrees, speed)[0];
        double y = getXY(degrees, speed)[1];
        if (uSside == PublicEnums.USside.LEFT) {
            while (leftBeaconUS.getUltrasonicLevel() < target && opMode.opModeIsActive()) {
                leftFrontMotor.setPower(((-x + y) / 2));
                leftBackMotor.setPower(((-x - y) / 2));
                rightFrontMotor.setPower(((x + y) / 2));
                rightBackMotor.setPower(((x - y) / 2));

                opMode.telemetry.addData("LeftUS:", leftBeaconUS.getUltrasonicLevel());
                opMode.telemetry.update();
            }
        } else if (uSside == PublicEnums.USside.RIGHT) {
            while (rightBeaconUS.getUltrasonicLevel() < target && opMode.opModeIsActive()) {
                leftFrontMotor.setPower(((-x + y) / 2));
                leftBackMotor.setPower(((-x - y) / 2));
                rightFrontMotor.setPower(((x + y) / 2));
                rightBackMotor.setPower(((x - y) / 2));

                opMode.telemetry.addData("RightUS:", rightBeaconUS.getUltrasonicLevel());
                opMode.telemetry.update();
            }
        }
        motorStop();
    }

    public void goToStartPosition(PublicEnums.AllianceColor allianceColor) {
        directionChange(PublicEnums.Direction.E);
        double targetDist = RobotVals.START_POST;
        double startDist;
        if (allianceColor == PublicEnums.AllianceColor.RED) {
            startDist = leftBeaconUS.getUltrasonicLevel();
            if (startDist < targetDist) {
                driveToUSPost(180, 0.7, targetDist, PublicEnums.USside.LEFT);
            } else if (startDist > targetDist) {
                driveToUSPost(0, 0.7, targetDist, PublicEnums.USside.LEFT);
            }
        } else {
            startDist = rightBeaconUS.getUltrasonicLevel();
            if (startDist < targetDist) {
                driveToUSPost(180, 0.7, targetDist, PublicEnums.USside.RIGHT);
            } else if (startDist > targetDist) {
                driveToUSPost(0, 0.7, targetDist, PublicEnums.USside.RIGHT);
            }
        }
    }

    ////////////////////////////////////////////////////////
    //                   Line Programs                    //
    //                                                    //
    //                                                    //
    //                                                    //
    //                                                    //
    //                                                    //
    //                                                    //
    //                                                    //
    ////////////////////////////////////////////////////////
    public void driveToLine(double degrees, double speed, PublicEnums.Gyro gyro) throws InterruptedException {
        double x = getXY(degrees, speed)[0];
        double y = getXY(degrees, speed)[1];
        double currentHead = startDirection;
        while (isThereMat() && opMode.opModeIsActive()) {
            if (gyro == PublicEnums.Gyro.YES) {
                if (robotGS.getHeading() < currentHead) {
                    leftFrontMotor.setPower(((-x + y) / 2));
                    leftBackMotor.setPower(((-x - y) / 2));
                    rightFrontMotor.setPower(((x + y) / 2) / 2);
                    rightBackMotor.setPower(((x - y) / 2) / 2);
                } else if (robotGS.getHeading() > currentHead) {
                    leftFrontMotor.setPower(((-x + y) / 2) / 2);
                    leftBackMotor.setPower(((-x - y) / 2) / 2);
                    rightFrontMotor.setPower(((x + y) / 2));
                    rightBackMotor.setPower(((x - y) / 2));
                } else {
                    leftFrontMotor.setPower(((-x + y) / 2));
                    leftBackMotor.setPower(((-x - y) / 2));
                    rightFrontMotor.setPower(((x + y) / 2));
                    rightBackMotor.setPower(((x - y) / 2));
                }
                opMode.telemetry.addData("Robot Heading", robotGS.getHeading());
                opMode.telemetry.update();
            } else {

                leftFrontMotor.setPower(((-x + y) / 2));
                leftBackMotor.setPower(((-x - y) / 2));
                rightFrontMotor.setPower(((x + y) / 2));
                rightBackMotor.setPower(((x - y) / 2));
                opMode.telemetry.addData("Robot Heading", robotGS.getHeading());
                opMode.telemetry.update();
            }
        }
//        if (degrees>90 && degrees <= 270) {
//            double x = getXY(degrees, speed)[0];
//            double y = getXY(degrees, speed)[1];
//            double currentHead = startDirection;
//            while (isThereMat() && opMode.opModeIsActive()) {
//                if (robotGS.getHeading() < currentHead) {
//                    leftFrontMotor.setPower(((-x + y) / 2));
//                    leftBackMotor.setPower(((-x - y) / 2));
//                    rightFrontMotor.setPower(((x + y) / 2) / 1.5);
//                    rightBackMotor.setPower(((x - y) / 2) / 1.5);
//                } else if (robotGS.getHeading() > currentHead) {
//                    leftFrontMotor.setPower(((-x + y) / 2) / 1.5);
//                    leftBackMotor.setPower(((-x - y) / 2) / 1.5);
//                    rightFrontMotor.setPower(((x + y) / 2));
//                    rightBackMotor.setPower(((x - y) / 2));
//                } else {
//                    leftFrontMotor.setPower(((-x + y) / 2));
//                    leftBackMotor.setPower(((-x - y) / 2));
//                    rightFrontMotor.setPower(((x + y) / 2));
//                    rightBackMotor.setPower(((x - y) / 2));
//                }
//                opMode.telemetry.addData("Robot Heading", robotGS.getHeading());
//                opMode.telemetry.update();
//            }
////        while (isThereMat() && opMode.opModeIsActive()) {
////
////            leftFrontMotor.setPower(((-x + y) / 2));
////            leftBackMotor.setPower(((-x - y) / 2));
////            rightFrontMotor.setPower(((x + y) / 2));
////            rightBackMotor.setPower(((x - y) / 2));
////            opMode.telemetry.addData("LF", ((-x + y) / 2));
////            opMode.telemetry.addData("LB", ((-x - y) / 2));
////            opMode.telemetry.addData("RF", ((x + y) / 2));
////            opMode.telemetry.addData("RB", ((x - y) / 2));
////            opMode.telemetry.addData("ODS light val:", lineODS.getLightDetected());
////            opMode.telemetry.update();
////        }
//
//        } else {
//            double x = getXY(degrees, speed)[0];
//            double y = getXY(degrees, speed)[1];
//            double currentHead = startDirection;
////        while (isThereMat() && opMode.opModeIsActive()) {
////
////            leftFrontMotor.setPower(((-x + y) / 2));
////            leftBackMotor.setPower(((-x - y) / 2));
////            rightFrontMotor.setPower(((x + y) / 2));
////            rightBackMotor.setPower(((x - y) / 2));
////            opMode.telemetry.addData("LF", ((-x + y) / 2));
////            opMode.telemetry.addData("LB", ((-x - y) / 2));
////            opMode.telemetry.addData("RF", ((x + y) / 2));
////            opMode.telemetry.addData("RB", ((x - y) / 2));
////            opMode.telemetry.addData("ODS light val:", lineODS.getLightDetected());
////            opMode.telemetry.update();
////        }
//
//            while (isThereMat() && opMode.opModeIsActive()) {
//                if (robotGS.getHeading() > currentHead) {
//                    leftFrontMotor.setPower(((-x + y) / 2));
//                    leftBackMotor.setPower(((-x - y) / 2));
//                    rightFrontMotor.setPower(((x + y) / 2) / 1.5);
//                    rightBackMotor.setPower(((x - y) / 2) / 1.5);
//                } else if (robotGS.getHeading() < currentHead) {
//                    leftFrontMotor.setPower(((-x + y) / 2) / 1.5);
//                    leftBackMotor.setPower(((-x - y) / 2) / 1.5);
//                    rightFrontMotor.setPower(((x + y) / 2));
//                    rightBackMotor.setPower(((x - y) / 2));
//                } else {
//                    leftFrontMotor.setPower(((-x + y) / 2));
//                    leftBackMotor.setPower(((-x - y) / 2));
//                    rightFrontMotor.setPower(((x + y) / 2));
//                    rightBackMotor.setPower(((x - y) / 2));
//                }
//                opMode.telemetry.addData("Robot Heading", robotGS.getHeading());
//                opMode.telemetry.update();
//            }
//        }
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);

    }

    public double getGroundLight() {
        return lineODS.getLightDetected();
    }

    public boolean isThereMat() {

        if (getGroundLight() - .3 <= matColorVal) {
            return true;
        } else {
            return false;
        }
    }

    ////////////////////////////////////////////////////////
    //                  Beacon Programs                   //
    //                                                    //
    //                                                    //
    //                                                    //
    //                                                    //
    //                                                    //
    //                                                    //
    //                                                    //
    ////////////////////////////////////////////////////////
    public void beaconRed() throws InterruptedException {
        beaconCS.enableLed(false);
        int x = 0;
        while (opMode.opModeIsActive() && x < 1) {
            if (beaconCS.red() > beaconCS.blue()) {
                beaconLeft.setPosition(RobotVals.BEACON_LEFT_FORWARD);
                Thread.sleep(1000);
                x++;

            } else if (beaconCS.blue() > beaconCS.red()) {
                beaconRight.setPosition(RobotVals.BEACON_RIGHT_FORWARD);
                Thread.sleep(1000);
                x++;
            } else {
                beaconServoReset();
            }
        }
        beaconServoReset();
    }

    public void beaconServoReset() {
        beaconRight.setPosition(RobotVals.BEACON_RIGHT_BACK);
        beaconLeft.setPosition(RobotVals.BEACON_LEFT_BACK);
    }

    public void beaconBlue() throws InterruptedException {
        beaconCS.enableLed(false);
        int x = 0;
        while (opMode.opModeIsActive() && x < 1) {
            if (beaconCS.red() < beaconCS.blue()) {
                beaconLeft.setPosition(RobotVals.BEACON_LEFT_FORWARD);
                Thread.sleep(1000);
                x++;

            } else if (beaconCS.blue() < beaconCS.red()) {
                beaconRight.setPosition(RobotVals.BEACON_RIGHT_FORWARD);
                Thread.sleep(1000);
                x++;
            } else {
                beaconServoReset();
            }
        }
        beaconServoReset();
    }

    public void shoot(int times) throws InterruptedException {
        rightShooter.setPower(RobotVals.SHOOTER_SPEED);
        leftShooter.setPower(RobotVals.SHOOTER_SPEED);
        Thread.sleep(2000);
        for (int i = 0; i < times; i++) {
            shoot.setPosition(RobotVals.SHOOTER_UP);
            Thread.sleep(1000);
            shoot.setPosition(RobotVals.SHOOTER_DOWN);
            Thread.sleep(1000);

        }
        rightShooter.setPower(0);
        leftShooter.setPower(0);
    }
}



