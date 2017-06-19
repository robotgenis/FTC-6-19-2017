package com.pineapplerobotics.velocityvortex.javacodeclasses.resources;

import com.pineapplerobotics.velocityvortex.javacodeclasses.resources.RobotVals;
import com.pineapplerobotics.velocityvortex.javacodeclasses.resources.enums.PublicEnums;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 */
public class RobotHardware {
    /* Public OpMode members. */
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor rightBackMotor = null;
    //    public DcMotor yAxisEncoder = null;
//    public DcMotor xAxisEncoder = null;
    public DcMotor rightLifter = null;
    public DcMotor leftLifter = null;

    public DcMotor rightShooter = null;
    public DcMotor leftShooter = null;

    public PublicEnums.Direction robotDirection = PublicEnums.Direction.N;
    public Servo beaconRight = null;
    public Servo beaconLeft = null;
    public Servo lock = null;
    public Servo shoot = null;

    public CRServo lift = null;

    public ColorSensor beaconCS = null;

    public OpticalDistanceSensor lineODS = null;

    public GyroSensor robotGS = null;

    public double matColorVal = 0;

    public int startDirection = 0;
    public TouchSensor t = null;

    public UltrasonicSensor rightBeaconUS = null;
    public UltrasonicSensor leftBeaconUS = null;
    /* local OpMode members. */


    LinearOpMode opMode;


    /* Constructor */
    public RobotHardware(LinearOpMode opMode) {
        this.opMode = opMode;
        // Define and Initialize
        leftFrontMotor = opMode.hardwareMap.dcMotor.get("left_front");
        rightFrontMotor = opMode.hardwareMap.dcMotor.get("right_front");
        leftBackMotor = opMode.hardwareMap.dcMotor.get("left_back");
        rightBackMotor = opMode.hardwareMap.dcMotor.get("right_back");
        //leftLifter = opMode.hardwareMap.dcMotor.get("leftLifter");
        //rightLifter = opMode.hardwareMap.dcMotor.get("rightLifter");
//lifter = opMode.hardwareMap.dcMotor.get("lifter");
//        xAxisEncoder = opMode.hardwareMap.dcMotor.get("xEncode");
//        yAxisEncoder = opMode.hardwareMap.dcMotor.get("yEncode");
        shoot = opMode.hardwareMap.servo.get("s");
        rightShooter = opMode.hardwareMap.dcMotor.get("rightShooter");
        leftShooter = opMode.hardwareMap.dcMotor.get("leftShooter");

        lift = opMode.hardwareMap.crservo.get("cs");

        beaconRight = opMode.hardwareMap.servo.get("beaconRight");
        beaconLeft = opMode.hardwareMap.servo.get("beaconLeft");
        //lock = opMode.hardwareMap.servo.get("lock");

        beaconCS = opMode.hardwareMap.colorSensor.get("bCS");
        lineODS = opMode.hardwareMap.opticalDistanceSensor.get("lODS");

        //robotGS = opMode.hardwareMap.gyroSensor.get("RobotGS");

        //t = opMode.hardwareMap.touchSensor.get("t");
        rightBeaconUS = opMode.hardwareMap.ultrasonicSensor.get("rightBeaconUS");
        leftBeaconUS = opMode.hardwareMap.ultrasonicSensor.get("leftBeaconUS");

    }

    public void initRobot() throws InterruptedException {


        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftLifter.setDirection(DcMotor.Direction.REVERSE);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        // Set all motors to zero power
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightLifter.setPower(0);
        leftLifter.setPower(0);
//lifter.setPower(0);
        beaconServoReset();
        lock.setPosition(1);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        yAxisEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        xAxisEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        beaconCS.enableLed(false);
        lineODS.enableLed(true);

        matColorVal = lineODS.getLightDetected();
        //   robotGS.calibrate();

        startDirection = robotGS.getHeading();
    }

    public void directionChange(PublicEnums.Direction direction) {
        robotDirection = direction;

        switch (direction) {
            case N:
                leftFrontMotor = opMode.hardwareMap.dcMotor.get("left_front");
                rightFrontMotor = opMode.hardwareMap.dcMotor.get("right_front");
                leftBackMotor = opMode.hardwareMap.dcMotor.get("left_back");
                rightBackMotor = opMode.hardwareMap.dcMotor.get("right_back");
                opMode.telemetry.addData("Robot Direction:", "N");
                break;
            case E:
                leftFrontMotor = opMode.hardwareMap.dcMotor.get("left_back");
                rightFrontMotor = opMode.hardwareMap.dcMotor.get("left_front");
                leftBackMotor = opMode.hardwareMap.dcMotor.get("right_back");
                rightBackMotor = opMode.hardwareMap.dcMotor.get("right_front");
                opMode.telemetry.addData("Robot Direction:", "E");

                break;
            case S:
                leftFrontMotor = opMode.hardwareMap.dcMotor.get("right_back");
                rightFrontMotor = opMode.hardwareMap.dcMotor.get("left_back");
                leftBackMotor = opMode.hardwareMap.dcMotor.get("right_front");
                rightBackMotor = opMode.hardwareMap.dcMotor.get("left_front");
                opMode.telemetry.addData("Robot Direction:", "S");

                break;
            case W:
                leftFrontMotor = opMode.hardwareMap.dcMotor.get("right_front");
                rightFrontMotor = opMode.hardwareMap.dcMotor.get("right_back");
                leftBackMotor = opMode.hardwareMap.dcMotor.get("left_front");
                rightBackMotor = opMode.hardwareMap.dcMotor.get("left_back");
                opMode.telemetry.addData("Robot Direction:", "W");

                break;
            default:
                leftFrontMotor = opMode.hardwareMap.dcMotor.get("left_front");
                rightFrontMotor = opMode.hardwareMap.dcMotor.get("right_front");
                leftBackMotor = opMode.hardwareMap.dcMotor.get("left_back");
                rightBackMotor = opMode.hardwareMap.dcMotor.get("right_back");
                opMode.telemetry.addData("Robot Direction:", "N");
                break;

        }
        opMode.telemetry.update();
    }

    public boolean encoderReached(int targetPositionLB, int targetPositionLF, int targetPositionRB, int targetPositionRF) {
        if (leftBackMotor.getCurrentPosition() < targetPositionLB && leftFrontMotor.getCurrentPosition() < targetPositionLF && rightBackMotor.getCurrentPosition() < targetPositionRB && rightFrontMotor.getCurrentPosition() < targetPositionRF) {
            return false;
        } else {
            return true;
        }

    }

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

    public double getRobotHeading() {
        return robotGS.getHeading();
    }

    public void basicLineFollow() {


        if (isColorWhite() == true) {
            leftFrontMotor.setPower(-0.3);
            leftBackMotor.setPower(-0.3);
        } else {
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
        }
        if (isColorWhite() != true) {
            rightFrontMotor.setPower(0.3);
            rightBackMotor.setPower(0.3);
        } else {
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
        }
    }

    public void advancedLineFollow() {
        double y = averageColor() - 0.25;
        double motorRightalf = y;
        double motorLeftalf = 1 - y;


        motorRightalf = -motorRightalf;

        leftFrontMotor.setPower(-motorRightalf / 6);
        leftBackMotor.setPower(motorLeftalf / 6);

        rightFrontMotor.setPower(-motorLeftalf / 6);
        rightBackMotor.setPower(motorRightalf / 6);
    }

//    public boolean isColorBlack() {
//        double x = 0.7;
//        if (lineODS.getLightDetected()<x) {
//            return true;
//        } else {
//            return false;
//        }
//    }

    public double averageColor() {
        double x;
        x = lineODS.getLightDetected();
        return x;
    }

    public boolean isColorWhite() {
        double x = 0.7;

        if (lineODS.getLightDetected() >= x) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isThereMat() {

        if (getGroundLight() - .2 <= matColorVal) {
            return true;
        } else {
            return false;
        }
    }

    public void encoderDrive(double speed, double inches_x, double inches_y) throws InterruptedException {
        int newLeftFTarget;
        int newLeftBTarget;
        int newRightFTarget;
        int newRightBTarget;

        double x = getXY(getAngleOfInc(inches_x, inches_y), speed)[0];
        double y = getXY(getAngleOfInc(inches_x, inches_y), speed)[1];
//        robotGS.calibrate();
//        while (robotGS.isCalibrating()) {
//            Thread.sleep(50);
//        }

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        newLeftFTarget = leftFrontMotor.getCurrentPosition() + (int) (((-inches_x + inches_y) / 2) * RobotVals.COUNTS_PER_INCH_NEVEREST);
        newLeftBTarget = leftBackMotor.getCurrentPosition() + (int) (((-inches_x - inches_y) / 2) * RobotVals.COUNTS_PER_INCH_NEVEREST);
        newRightFTarget = rightFrontMotor.getCurrentPosition() + (int) (((inches_x + inches_y) / 2) * RobotVals.COUNTS_PER_INCH_NEVEREST);
        newRightBTarget = rightBackMotor.getCurrentPosition() + (int) (((inches_x - inches_y) / 2) * RobotVals.COUNTS_PER_INCH_NEVEREST);

        leftFrontMotor.setTargetPosition(newLeftFTarget);
        leftBackMotor.setTargetPosition(newLeftBTarget);
        rightFrontMotor.setTargetPosition(newRightFTarget);
        rightBackMotor.setTargetPosition(newRightBTarget);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (!encoderReached(newLeftBTarget, newLeftFTarget, newRightBTarget, newRightFTarget)) {
            opMode.telemetry.addData("gyro", getRobotHeading());

            if (robotGS.getHeading() < 0) {
                leftFrontMotor.setPower(((-x + y) / 2));
                leftBackMotor.setPower(((-x - y) / 2));
                rightFrontMotor.setPower(((x + y) / 2) / 1.5);
                rightBackMotor.setPower(((x - y) / 2) / 1.5);
            } else if (robotGS.getHeading() > 0) {
                leftFrontMotor.setPower(((-x + y) / 2) / 1.5);
                leftBackMotor.setPower(((-x - y) / 2) / 1.5);
                rightFrontMotor.setPower(((x + y) / 2));
                rightBackMotor.setPower(((x - y) / 2));
            } else {
                leftFrontMotor.setPower(((-x + y) / 2));
                leftBackMotor.setPower(((-x - y) / 2));
                rightFrontMotor.setPower(((x + y) / 2));
                rightBackMotor.setPower(((x - y) / 2));
            }

        }
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void gyroTurn(double degrees) throws InterruptedException {
        robotGS.calibrate();
        while (robotGS.isCalibrating()) {
            Thread.sleep(50);
        }
        int x = 0;
        double targetHeading = degrees;  //This need to change and fixed to be exactly 90 degree tur
        int currentHeading = 0;

        robotGS.resetZAxisIntegrator();

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        Thread.sleep(1000);

        if (degrees <= 180) {
            while (x < 1) {

                if (currentHeading > targetHeading + 5) {
                    leftFrontMotor.setPower(-0.1);
                    leftBackMotor.setPower(-0.1);
                    rightFrontMotor.setPower(-0.1);
                    rightBackMotor.setPower(-0.1);
                }

                if (currentHeading < targetHeading - 5) {
                    leftFrontMotor.setPower(0.1);
                    leftBackMotor.setPower(0.1);
                    rightFrontMotor.setPower(0.1);
                    rightBackMotor.setPower(0.1);
                }
                if (currentHeading > targetHeading && currentHeading <= targetHeading + 5) {
                    leftFrontMotor.setPower(-0.07);
                    leftBackMotor.setPower(-0.07);
                    rightFrontMotor.setPower(-0.07);
                    rightBackMotor.setPower(-0.07);
                }

                if (currentHeading < targetHeading && currentHeading >= targetHeading - 5) {
                    leftFrontMotor.setPower(0.07);
                    leftBackMotor.setPower(0.07);
                    rightFrontMotor.setPower(0.07);
                    rightBackMotor.setPower(0.07);
                } else {
                    leftFrontMotor.setPower(0);
                    leftBackMotor.setPower(0);
                    rightFrontMotor.setPower(0);
                    rightBackMotor.setPower(0);
                    x++;
                }

                currentHeading = robotGS.getHeading();

                Thread.sleep(10);
            }
        } else {
            while (x < 1) {
                targetHeading = 360 - targetHeading;

                if (currentHeading > targetHeading + 5) {
                    leftFrontMotor.setPower(0.1);
                    leftBackMotor.setPower(0.1);
                    rightFrontMotor.setPower(0.1);
                    rightBackMotor.setPower(0.1);
                } else if (currentHeading < targetHeading - 5) {
                    leftFrontMotor.setPower(-0.1);
                    leftBackMotor.setPower(-0.1);
                    rightFrontMotor.setPower(-0.1);
                    rightBackMotor.setPower(-0.1);
                } else if (currentHeading > targetHeading && currentHeading <= targetHeading + 5) {
                    leftFrontMotor.setPower(0.07);
                    leftBackMotor.setPower(0.07);
                    rightFrontMotor.setPower(0.07);
                    rightBackMotor.setPower(0.07);
                } else if (currentHeading < targetHeading && currentHeading >= targetHeading - 5) {
                    leftFrontMotor.setPower(-0.07);
                    leftBackMotor.setPower(-0.07);
                    rightFrontMotor.setPower(-0.07);
                    rightBackMotor.setPower(-0.07);
                } else {
                    leftFrontMotor.setPower(0);
                    leftBackMotor.setPower(0);
                    rightFrontMotor.setPower(0);
                    rightBackMotor.setPower(0);
                    x++;
                }

                currentHeading = robotGS.getHeading();

                Thread.sleep(10);
            }
        }
    }

    public void checkDirection() {
        if (robotGS.getHeading() > startDirection) {
            leftFrontMotor.setPower(.1);
            leftBackMotor.setPower(-.1);
            rightFrontMotor.setPower(-.1);
            rightBackMotor.setPower(.1);
        } else if (robotGS.getHeading() < startDirection) {
            leftFrontMotor.setPower(.1);
            leftBackMotor.setPower(-.1);
            rightFrontMotor.setPower(-.1);
            rightBackMotor.setPower(.1);
        }
        while (startDirection != robotGS.getHeading()) {
        }
    }

    public void driveToLine(double degrees, double speed, PublicEnums.Gyro gyro) throws InterruptedException {
        double x = getXY(degrees, speed)[0];
        double y = getXY(degrees, speed)[1];
        double currentHead = startDirection;
        while (isThereMat() && opMode.opModeIsActive()) {
            if (gyro == PublicEnums.Gyro.YES) {
                if (robotGS.getHeading() < currentHead) {
                    leftFrontMotor.setPower(((-x + y) / 2));
                    leftBackMotor.setPower(((-x - y) / 2));
                    rightFrontMotor.setPower(((x + y) / 2) / 1.5);
                    rightBackMotor.setPower(((x - y) / 2) / 1.5);
                } else if (robotGS.getHeading() > currentHead) {
                    leftFrontMotor.setPower(((-x + y) / 2) / 1.5);
                    leftBackMotor.setPower(((-x - y) / 2) / 1.5);
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

    public void gyroAlign(double degrees) {
        double currentDirection;
        double targetDirection = degrees;
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

    public double getAngleOfInc(double x, double y) {
        double angle;

        if (x > 0 && y > 0) {
            angle = 90 - Math.toDegrees(Math.atan(Math.abs(y) / Math.abs(x)));
        } else if (x > 0 && y < 0) {
            angle = 180 - Math.toDegrees(Math.atan(Math.abs(y) / Math.abs(x)));
        } else if (x < 0 && y > 0) {
            angle = 360 - Math.toDegrees(Math.atan(Math.abs(y) / Math.abs(x)));

        } else if (x < 0 && y < 0) {
            angle = 270 - Math.toDegrees(Math.atan(Math.abs(y) / Math.abs(x)));

        } else if (x == 0 && y > 0) {
            angle = 0;
        } else if (x == 0 && y < 0) {
            angle = 180;
        } else if (y == 0 && x > 0) {
            angle = 90;
        } else {
            angle = 270;
        }

        return angle;
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

    public boolean isThereBeacon() {
        if (beaconCS.blue() > 1 || beaconCS.red() > 1) {
            return true;
        } else {
            return false;
        }
    }

    public static double ultraSonicDist(UltrasonicSensor US) {
        return US.getUltrasonicLevel();
    }

    public double getGroundLight() {
        return lineODS.getLightDetected();
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

//    public void stopDrive(double x, double y) {
//        xAxisEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        yAxisEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        double xTarget = x * RobotVals.COUNTS_PER_INCH_TETRIX;
//        double yTarget = y * RobotVals.COUNTS_PER_INCH_TETRIX;
//        double xCurrent;
//        double yCurrent;
//        double xDiff;
//        double yDiff;
//        double xDiffPast = -1;
//        double yDiffPast = -1;
//        int loopBreak = 0;
//        while (opMode.opModeIsActive() && loopBreak < 1) {
//            xCurrent = xAxisEncoder.getCurrentPosition();
//            yCurrent = yAxisEncoder.getCurrentPosition();
//            xDiff = xTarget - xCurrent;
//            yDiff = yTarget - yCurrent;
//            if (xDiffPast > xDiff || xCurrent != xTarget || yDiffPast > yDiff || yCurrent != yTarget) {
//
//            } else {
//                loopBreak = 1;
//                motorStop();
//            }
//            xDiffPast = xDiff;
//            yDiffPast = yDiff;
//        }
//    }

    public void beaconServoReset() {
        beaconRight.setPosition(RobotVals.BEACON_RIGHT_BACK);
        beaconLeft.setPosition(RobotVals.BEACON_LEFT_BACK);
    }

    public void motorStop() {
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }

    public void alignWithWallUS() {
        double startDistR = rightBeaconUS.getUltrasonicLevel();
        double startDistL = leftBeaconUS.getUltrasonicLevel();
        if (startDistL > startDistR) {
            while (!(startDistL == startDistR)) {
                leftBackMotor.setPower(0.2);
                leftFrontMotor.setPower(0.2);
                rightBackMotor.setPower(0.2);
                rightFrontMotor.setPower(0.2);
            }
        } else if (startDistL < startDistR) {
            while (!(startDistL == startDistR)) {
                leftBackMotor.setPower(-0.2);
                leftFrontMotor.setPower(-0.2);
                rightBackMotor.setPower(-0.2);
                rightFrontMotor.setPower(-0.2);
            }
        }
    }

//    public void liftToTop() throws InterruptedException {
//        rightLifter.setPower(1);
//        leftLifter.setPower(1);
//        Thread.sleep((long) (RobotVals.MILLISECONDS_TO_LIFT));
//        rightLifter.setPower(0);
//        leftLifter.setPower(0);
//    }
//
//    public void liftToBottem() throws InterruptedException {
//        rightLifter.setPower(-1);
//        leftLifter.setPower(-1);
//
//        Thread.sleep((long) (RobotVals.MILLISECONDS_TO_LIFT));
//        rightLifter.setPower(0);
//        leftLifter.setPower(0);
//    }
}



