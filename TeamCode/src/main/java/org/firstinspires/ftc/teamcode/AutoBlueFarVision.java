/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.OpenCV.TeamElementSubsystem;

//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoBlueFarVision")
public class AutoBlueFarVision extends LinearOpMode {
    IMU imu;
    /* Declare OpMode members. */
    Robot myRobot = new Robot();

    ElapsedTime mStateTime = new ElapsedTime();
    boolean outtakeArmUp = false;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = 0.2;

    String parking = "corner";
    String route = "short";
    long initDelay = 0000;
    boolean addInitDelay;
    double backDropTargetDist = 1.6;


    static final double SPEED_SLOW  = 0.1;
    static final double SPEED_MODERATE = 0.4;
    static final double SPEED_FAST  = 0.7;
    static final double SPEED_FULLSPEED = 1.0;

    YawPitchRollAngles orientation;
    String curAlliance = "blue";
    public int element_zone = 2;

    private TeamElementSubsystem teamElementDetection=null;

    boolean togglePreview = true;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        teamElementDetection = new TeamElementSubsystem(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }

    @Override
    public void runOpMode() {



        //initialize imu
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

        imu.initialize(parameters);

        orientation = imu.getRobotYawPitchRollAngles();
        //YawPitchRollAngles orientation= imu.getRobotYawPitchRollAngles();;
        double yaw = orientation.getYaw(AngleUnit.DEGREES);

        telemetry.addData("Initial yaw", yaw);

        //telemetry.update();

        /*
         * Initialize the drive system variables.
         */


        DcMotor leftFrontMotor;
        DcMotor rightFrontMotor;
        DcMotor leftBackMotor;
        DcMotor rightBackMotor;
        CRServo outtakeWheel;
        DcMotor slideMotor1;
        DcMotor slideMotor2;
        Servo elbow;
        Servo wrist;
        Servo outtakeDoor;
        DcMotor intake;
        DistanceSensor DstSensor;

        leftFrontMotor = hardwareMap.get(DcMotor.class, "LF");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RF");
        leftBackMotor = hardwareMap.get(DcMotor.class, "LB");
        rightBackMotor = hardwareMap.get(DcMotor.class, "RB");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class,"intake");
        slideMotor1 = hardwareMap.get(DcMotor.class,"slide_motor1");
        slideMotor2 = hardwareMap.get(DcMotor.class,"slide_motor2");
        outtakeWheel = hardwareMap.get(CRServo.class, "outtakeWheel");
        wrist = hardwareMap.get(Servo.class, "outtakeWrist");
        elbow = hardwareMap.get(Servo.class, "outtakeElbow");
        DstSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        double backDropDist;

        slideMotor1.setDirection(DcMotor.Direction.REVERSE);
        outtakeDoor = hardwareMap.get(Servo.class, "outtakeDoor");
        slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myRobot.setDriveTrainMode(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor, DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Resetting Encoders");
        myRobot.setDriveTrainMode(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("LF encoder before start",leftFrontMotor.getCurrentPosition());
        telemetry.addData("RF encoder before start",rightFrontMotor.getCurrentPosition());
        telemetry.addData("LB encoder before start",leftBackMotor.getCurrentPosition());
        telemetry.addData("RB encoder before start",rightBackMotor.getCurrentPosition());
        telemetry.update();

        sleep(1000);
        HardwareStart();

        //element_zone = teamElementDetection.ElementDetection(telemetry);

        while (!opModeIsActive() && !isStopRequested()){
            element_zone = teamElementDetection.elementDetection(telemetry);

            if (togglePreview && gamepad2.a){
                togglePreview = false;
                teamElementDetection.toggleAverageZone();
            }else if (!gamepad2.a){
                togglePreview = true;
            }

            if (gamepad1.y ){
                initDelay = 5000;
            }

            else if(gamepad1.a){
                initDelay = 0000;
            }


            telemetry.addData("Select initial Delay (Gamepad1 y = 5 seconds, Gamepad1 a = 0 seconds", "");
            telemetry.addData("Current initDelay:", initDelay);


            if (gamepad1.x){
                curAlliance = "blue";
            }else if (gamepad1.b){
                curAlliance = "red";
            }
            teamElementDetection.setAlliance(curAlliance);
            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());

            if (gamepad1.dpad_down){
                route = "short";
            }
            else if (gamepad1.dpad_up){
                route = "long";
            }

            telemetry.addData("Select route (Gamepad1 down = short, Gamepad1 up = long", "");
            telemetry.addData("Current Route Selected:", route);

            if (gamepad1.dpad_left){
                parking = "center";
            }
            else if (gamepad1.dpad_right){
                parking = "corner";
            }

            telemetry.addData("Select parking area (Gamepad1 left = center, Gamepad1 right = corner", "");
            telemetry.addData("Current parking area selected:", parking);


            telemetry.update();
        }

        telemetry.addData("Object", "Passed waitForStart");
        telemetry.addData("ColorZone", element_zone);
        telemetry.update();

        myRobot.OuttakeArmRest(elbow, wrist, outtakeDoor, outtakeWheel);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        sleep(initDelay);

        if (route == "short") {

            telemetry.addData("started short route", "");
            telemetry.update();

            if (element_zone == 3) {

                moveBackwardsEncoders(SPEED_MODERATE, 1, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                strafeLeftEncoders(SPEED_MODERATE, 12, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                telemetry.addLine("passed strafe right");
                moveBackwardsEncoders(SPEED_MODERATE, 20, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                telemetry.update();
                moveForwardEncoders(SPEED_MODERATE, 16, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                turnLeft(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor, 90, SPEED_MODERATE);
                moveBackwardsEncoders(SPEED_FAST, 91, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);

                strafeLeftEncoders(SPEED_FAST, 34 , leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                openOuttakeArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist);

                sleep(250);

                backDropDist = DstSensor.getDistance(DistanceUnit.INCH);
                telemetry.addData("backDropDist", backDropDist);
                telemetry.update();


                if (backDropDist != 0 && backDropDist < 25) {
                    double distanceToMove = backDropDist - backDropTargetDist;
                    moveBackwardsEncoders(SPEED_MODERATE, distanceToMove, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                }
                else {
                    moveBackwardsEncoders(0.25, 2, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                }

                scorePixAndCloseArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist, outtakeDoor, outtakeWheel, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                strafeRightEncoders(SPEED_FAST, 10 , leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);


            } else if (element_zone == 2) {
                moveBackwardsEncoders(SPEED_MODERATE, 28.5, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                moveForwardEncoders(SPEED_FAST, 23, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                turnLeft(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor, 90, 0.25);
                moveBackwardsEncoders(SPEED_FAST, 79, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);

                strafeLeftEncoders(SPEED_FAST, 25 , leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                openOuttakeArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist);

                sleep(250);

                backDropDist = DstSensor.getDistance(DistanceUnit.INCH);
                telemetry.addData("backDropDist", backDropDist);
                telemetry.update();


                if (backDropDist != 0 && backDropDist < 25) {
                    double distanceToMove = backDropDist - backDropTargetDist;
                    moveBackwardsEncoders(SPEED_MODERATE, distanceToMove, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                }
                else {
                    moveBackwardsEncoders(0.25, 2, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                }

                scorePixAndCloseArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist, outtakeDoor, outtakeWheel, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);


            } else if (element_zone == 1) {
                moveBackwardsEncoders(SPEED_MODERATE, 25, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                turnLeft(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor, 90, 0.4);
                moveBackwardsEncoders(SPEED_MODERATE, 4, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                strafeLeftEncoders(SPEED_MODERATE, 3 , leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                moveForwardEncoders(SPEED_MODERATE, 6, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                turnRight(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor, 90, SPEED_MODERATE);
                moveForwardEncoders(SPEED_MODERATE, 24, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                turnLeft(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor, 90, SPEED_MODERATE);
                moveBackwardsEncoders(SPEED_FAST, 81, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);

                strafeLeftEncoders(SPEED_FAST, 18 , leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                openOuttakeArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist);

                sleep(250);

                backDropDist = DstSensor.getDistance(DistanceUnit.INCH);
                telemetry.addData("backDropDist", backDropDist);
                telemetry.update();


                if (backDropDist != 0 && backDropDist < 25) {
                    double distanceToMove = backDropDist - backDropTargetDist;
                    moveBackwardsEncoders(SPEED_MODERATE, distanceToMove, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                }
                else {
                    moveBackwardsEncoders(0.25, 2, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                }

                scorePixAndCloseArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist, outtakeDoor, outtakeWheel, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                strafeLeftEncoders(SPEED_FAST, 12 , leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);

            }

            sleep(1000);

            if (parking == "corner") {
                strafeRightEncoders(SPEED_FAST, 26, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                moveBackwardsEncoders(SPEED_MODERATE, 12, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
            }

            else if (parking == "center") {
                strafeLeftEncoders(SPEED_FAST, 28, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                moveBackwardsEncoders(SPEED_MODERATE, 12, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
            }
        }

        else if (route == "long"){
            telemetry.addData("started long route", "");
            telemetry.update();

            //sleep(5000);

            if (element_zone == 3) {

                moveBackwardsEncoders(SPEED_MODERATE, 1, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                strafeLeftEncoders(SPEED_MODERATE, 13, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                telemetry.addLine("passed strafe left");
                moveBackwardsEncoders(SPEED_FAST, 20, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                telemetry.update();
                moveForwardEncoders(SPEED_FAST, 10, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                strafeRightEncoders(SPEED_MODERATE, 14, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                moveBackwardsEncoders(SPEED_FAST, 36.5, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                turnLeft(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor, 90, SPEED_MODERATE);
                moveBackwardsEncoders(SPEED_FULLSPEED, 70, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);

                strafeRightEncoders(SPEED_FAST, 20 , leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                openOuttakeArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist);

                sleep(250);

                backDropDist = DstSensor.getDistance(DistanceUnit.INCH);
                telemetry.addData("backDropDist", backDropDist);
                telemetry.update();


                if (backDropDist != 0 && backDropDist < 25) {
                    double distanceToMove = backDropDist - backDropTargetDist;
                    moveBackwardsEncoders(SPEED_MODERATE, distanceToMove, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                }
                else {
                    moveBackwardsEncoders(0.25, 2, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                }

                scorePixAndCloseArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist, outtakeDoor, outtakeWheel, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                strafeRightEncoders(SPEED_FAST, 11 , leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);


            } else if (element_zone == 2) {
                moveBackwardsEncoders(SPEED_MODERATE, 28.5, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                moveForwardEncoders(SPEED_FAST, 7, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                strafeLeftEncoders(SPEED_FAST, 19 , leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                moveBackwardsEncoders(SPEED_FAST, 30, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                turnLeft(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor, 90, SPEED_MODERATE);
                moveBackwardsEncoders(SPEED_FULLSPEED, 94, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                strafeRightEncoders(SPEED_FAST, 30 , leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                openOuttakeArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist);

                sleep(250);

                backDropDist = DstSensor.getDistance(DistanceUnit.INCH);
                telemetry.addData("backDropDist", backDropDist);
                telemetry.update();


                if (backDropDist != 0 && backDropDist < 25) {
                    double distanceToMove = backDropDist - backDropTargetDist;
                    moveBackwardsEncoders(SPEED_MODERATE, distanceToMove, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                }
                else {
                    moveBackwardsEncoders(0.25, 2, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                }

                scorePixAndCloseArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist, outtakeDoor, outtakeWheel, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);


            } else if (element_zone == 1) {
                moveBackwardsEncoders(SPEED_MODERATE, 25, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                turnLeft(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor, 90, 0.4);
                moveBackwardsEncoders(SPEED_MODERATE, 4, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                strafeLeftEncoders(SPEED_MODERATE, 3 , leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                moveForwardEncoders(SPEED_FAST, 10, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                strafeLeftEncoders(SPEED_FAST, 25 , leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                moveBackwardsEncoders(SPEED_FULLSPEED, 80, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);

                strafeRightEncoders(SPEED_FAST, 37 , leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                openOuttakeArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist);

                sleep(250);

                backDropDist = DstSensor.getDistance(DistanceUnit.INCH);
                telemetry.addData("backDropDist", backDropDist);
                telemetry.update();


                if (backDropDist != 0 && backDropDist < 25) {
                    double distanceToMove = backDropDist - backDropTargetDist;
                    moveBackwardsEncoders(SPEED_MODERATE, distanceToMove, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                }
                else {
                    moveBackwardsEncoders(0.25, 2, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                }

                scorePixAndCloseArm(SPEED_FAST, 600, slideMotor1, slideMotor2, elbow, wrist, outtakeDoor, outtakeWheel, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                strafeLeftEncoders(SPEED_FAST, 8 , leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);

            }

            sleep(1000);

            if (parking == "corner") {
                strafeRightEncoders(SPEED_FAST, 28, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                moveBackwardsEncoders(SPEED_MODERATE, 12, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
            }

            else if (parking == "center") {
                strafeLeftEncoders(SPEED_FAST, 28, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
                moveBackwardsEncoders(SPEED_MODERATE, 12, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
            }
        }

        /**
         * This is a test
         * customEncoderDrive(DRIVE_SPEED, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor,100, 100, 100, 100);
         telemetry.addData("LF encoder after move",leftFrontMotor.getCurrentPosition());
         telemetry.addData("RF encoder after move",rightFrontMotor.getCurrentPosition());
         telemetry.addData("LB encoder after move",leftBackMotor.getCurrentPosition());++
         telemetry.addData("RB encoder after move",rightBackMotor.getCurrentPosition());
         telemetry.update();
         //sleep(5000);


         //not working//turnLeft(orientation, leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, 90, 0.2);
         turnLeft( leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, 90, 0.2);
         telemetry.addData("Yaw after left turn", orientation.getYaw(AngleUnit.DEGREES));
         telemetry.update();
         //sleep(5000);

         turnRight( leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, 90, 0.2);
         telemetry.addData("Yaw after right turn", orientation.getYaw(AngleUnit.DEGREES));
         telemetry.update();
         //sleep(5000);
         */

    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void customEncoderDrive(double speed, DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack,
                                   double leftFront, double rightFront, double leftBack, double rightBack) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = DcleftFront.getCurrentPosition() + (int) (leftFront * COUNTS_PER_INCH);
            newRFTarget = DcrightFront.getCurrentPosition() + (int) (rightFront * COUNTS_PER_INCH);
            newLBTarget = DcleftBack.getCurrentPosition() + (int) (leftBack * COUNTS_PER_INCH);
            newRBTarget = DcrightBack.getCurrentPosition() + (int) (rightBack * COUNTS_PER_INCH);

            telemetry.addData("newLFTarget", newLFTarget);
            telemetry.addData("newRFTarget", newRFTarget);
            telemetry.addData("newLBTarget", newLBTarget);
            telemetry.addData("newRBTarget", newRBTarget);
            telemetry.update();
            //sleep(5000);

            DcleftFront.setTargetPosition(newLFTarget);
            DcrightFront.setTargetPosition(newRFTarget);
            DcleftBack.setTargetPosition(newLBTarget);
            DcrightBack.setTargetPosition(newRBTarget);

            // Turn On RUN_TO_POSITION
            myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            DcleftFront.setPower(Math.abs(speed));
            DcrightFront.setPower(Math.abs(speed));
            DcleftBack.setPower(Math.abs(speed));
            DcrightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (DcleftFront.isBusy() && DcrightFront.isBusy()
                            && DcleftBack.isBusy() && DcrightBack.isBusy())) {
            }

            // Stop all motion;
            DcleftFront.setPower(0);
            DcrightFront.setPower(0);
            DcleftBack.setPower(0);
            DcrightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void moveForwardEncoders(double speed, double inches, DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = DcleftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRFTarget = DcrightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newLBTarget = DcleftBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRBTarget = DcrightBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            telemetry.addData("newLFTarget", newLFTarget);
            telemetry.addData("newRFTarget", newRFTarget);
            telemetry.addData("newLBTarget", newLBTarget);
            telemetry.addData("newRBTarget", newRBTarget);
            telemetry.update();
            //sleep(5000);

            DcleftFront.setTargetPosition(newLFTarget);
            DcrightFront.setTargetPosition(newRFTarget);
            DcleftBack.setTargetPosition(newLBTarget);
            DcrightBack.setTargetPosition(newRBTarget);

            // Turn On RUN_TO_POSITION
            myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            DcleftFront.setPower(Math.abs(speed));
            DcrightFront.setPower(Math.abs(speed));
            DcleftBack.setPower(Math.abs(speed));
            DcrightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (DcleftFront.isBusy() && DcrightFront.isBusy()
                            && DcleftBack.isBusy() && DcrightBack.isBusy())) {
            }

            // Stop all motion;
            DcleftFront.setPower(0);
            DcrightFront.setPower(0);
            DcleftBack.setPower(0);
            DcrightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void moveBackwardsEncoders(double speed, double inches, DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = DcleftFront.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newRFTarget = DcrightFront.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newLBTarget = DcleftBack.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newRBTarget = DcrightBack.getCurrentPosition() + (int) -(inches * COUNTS_PER_INCH);

            telemetry.addData("newLFTarget", newLFTarget);
            telemetry.addData("newRFTarget", newRFTarget);
            telemetry.addData("newLBTarget", newLBTarget);
            telemetry.addData("newRBTarget", newRBTarget);
            telemetry.update();
            //sleep(5000);

            DcleftFront.setTargetPosition(newLFTarget);
            DcrightFront.setTargetPosition(newRFTarget);
            DcleftBack.setTargetPosition(newLBTarget);
            DcrightBack.setTargetPosition(newRBTarget);

            // Turn On RUN_TO_POSITION
            myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            DcleftFront.setPower(Math.abs(-speed));
            DcrightFront.setPower(Math.abs(-speed));
            DcleftBack.setPower(Math.abs(-speed));
            DcrightBack.setPower(Math.abs(-speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (DcleftFront.isBusy() && DcrightFront.isBusy()
                            && DcleftBack.isBusy() && DcrightBack.isBusy())) {
            }

            // Stop all motion;
            DcleftFront.setPower(0);
            DcrightFront.setPower(0);
            DcleftBack.setPower(0);
            DcrightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void strafeRightEncoders(double speed, double inches, DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = DcleftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRFTarget = DcrightFront.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newLBTarget = DcleftBack.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newRBTarget = DcrightBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            telemetry.addData("newLFTarget", newLFTarget);
            telemetry.addData("newRFTarget", newRFTarget);
            telemetry.addData("newLBTarget", newLBTarget);
            telemetry.addData("newRBTarget", newRBTarget);
            telemetry.update();
            //sleep(5000);

            DcleftFront.setTargetPosition(newLFTarget);
            DcrightFront.setTargetPosition(newRFTarget);
            DcleftBack.setTargetPosition(newLBTarget);
            DcrightBack.setTargetPosition(newRBTarget);

            // Turn On RUN_TO_POSITION
            myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            DcleftFront.setPower(Math.abs(speed));
            DcrightFront.setPower(Math.abs(-speed));
            DcleftBack.setPower(Math.abs(-speed));
            DcrightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (DcleftFront.isBusy() && DcrightFront.isBusy()
                            && DcleftBack.isBusy() && DcrightBack.isBusy())) {
            }

            // Stop all motion;
            DcleftFront.setPower(0);
            DcrightFront.setPower(0);
            DcleftBack.setPower(0);
            DcrightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void strafeLeftEncoders(double speed, double inches, DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = DcleftFront.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newRFTarget = DcrightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newLBTarget = DcleftBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRBTarget = DcrightBack.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);

            telemetry.addData("newLFTarget", newLFTarget);
            telemetry.addData("newRFTarget", newRFTarget);
            telemetry.addData("newLBTarget", newLBTarget);
            telemetry.addData("newRBTarget", newRBTarget);
            telemetry.update();
            //sleep(5000);

            DcleftFront.setTargetPosition(newLFTarget);
            DcrightFront.setTargetPosition(newRFTarget);
            DcleftBack.setTargetPosition(newLBTarget);
            DcrightBack.setTargetPosition(newRBTarget);

            // Turn On RUN_TO_POSITION
            myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            DcleftFront.setPower(Math.abs(-speed));
            DcrightFront.setPower(Math.abs(speed));
            DcleftBack.setPower(Math.abs(speed));
            DcrightBack.setPower(Math.abs(-speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (DcleftFront.isBusy() && DcrightFront.isBusy()
                            && DcleftBack.isBusy() && DcrightBack.isBusy())) {
            }

            // Stop all motion;
            DcleftFront.setPower(0);
            DcrightFront.setPower(0);
            DcleftBack.setPower(0);
            DcrightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void moveSlidesTime(double power, double timeMilliseconds, DcMotor slideMotor1, DcMotor slideMotor2){
        slideMotor1.setPower(-power);
        slideMotor2.setPower(-power);

        sleep((long) timeMilliseconds);

        slideMotor1.setPower(0.0);
        slideMotor2.setPower(0.0);
    }

    public void dropPixels(double slidesPower, double slidesTimeMilliseconds, DcMotor slideMotor1, DcMotor slideMotor2,
                           Servo elbow, Servo wrist, Servo outtakeDoor, CRServo outtakeWheel){
        moveSlidesTime(slidesPower, slidesTimeMilliseconds, slideMotor1, slideMotor2);
        sleep((long) slidesTimeMilliseconds + 200);
        myRobot.OuttakeArmDrop(elbow, wrist);
        sleep(1300);
        myRobot.OuttakeDoorOpen(outtakeDoor);
        myRobot.OuttakeWheelOn(outtakeWheel);
        sleep(800);
        moveSlidesTime(-slidesPower, slidesTimeMilliseconds, slideMotor1, slideMotor2);
        sleep(1000);
        myRobot.OuttakeArmRest(elbow, wrist, outtakeDoor, outtakeWheel);
    }

    public void dropPixelsTest(double slidesPower, double slidesTimeMilliseconds, DcMotor slideMotor1, DcMotor slideMotor2,
                               Servo elbow, Servo wrist, Servo outtakeDoor, CRServo outtakeWheel,
                               DcMotor leftFrontMotor, DcMotor rightFrontMotor, DcMotor leftBackMotor, DcMotor rightBackMotor){
        moveSlidesTime(slidesPower, slidesTimeMilliseconds, slideMotor1, slideMotor2);
        sleep((long) slidesTimeMilliseconds + 200);
        myRobot.OuttakeArmDrop(elbow, wrist);
        sleep(500);
        moveBackwardsEncoders(SPEED_MODERATE, 1.2, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
        sleep(1300);
        myRobot.OuttakeWheelReverse(outtakeWheel);
        sleep(800);
        moveSlidesTime(slidesPower,200, slideMotor1, slideMotor2);
        moveForwardEncoders(SPEED_MODERATE, 3, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
        moveSlidesTime(-slidesPower, slidesTimeMilliseconds + 200, slideMotor1, slideMotor2);
        sleep(1000);
        myRobot.OuttakeArmRest(elbow, wrist, outtakeDoor, outtakeWheel);
    }

    public void openOuttakeArm(double slidesPower, double slidesTimeMilliseconds, DcMotor slideMotor1, DcMotor slideMotor2,
                               Servo elbow, Servo wrist){
        moveSlidesTime(slidesPower, slidesTimeMilliseconds, slideMotor1, slideMotor2);
        sleep((long) slidesTimeMilliseconds + 200);
        outtakeArmUpV2(elbow, wrist);
        sleep(600);
    }

    public void scorePixAndCloseArm(double slidesPower, double slidesTimeMilliseconds, DcMotor slideMotor1, DcMotor slideMotor2,
                                    Servo elbow, Servo wrist, Servo outtakeDoor, CRServo outtakeWheel,
                                    DcMotor leftFrontMotor, DcMotor rightFrontMotor, DcMotor leftBackMotor, DcMotor rightBackMotor){
        myRobot.OuttakeWheelReverse(outtakeWheel);
        sleep(800);
        moveSlidesTime(slidesPower,200, slideMotor1, slideMotor2);
        moveForwardEncoders(SPEED_MODERATE, 3, leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
        moveSlidesTime(-slidesPower, slidesTimeMilliseconds + 200, slideMotor1, slideMotor2);
        sleep(1000);
        myRobot.OuttakeArmRest(elbow, wrist, outtakeDoor, outtakeWheel);
    }

    public void turnLeft( DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack,
                          double degrees, double power) {
        //public void turnLeft(YawPitchRollAngles orientation, DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack,
        //                   double degrees, double power) {

        //myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.resetYaw();

        //orientation=imu.getRobotYawPitchRollAngles();
        //telemetry.addData("Yaw outside loop", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Yaw outside left turn while loop", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("degree value", degrees);
        telemetry.update();
        //sleep(5000);

        while(opModeIsActive() && (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < degrees)) {
            // while(opModeIsActive() && (orientation.getYaw(AngleUnit.DEGREES) < degrees)) {
            //while(opModeIsActive()) {
            DcleftFront.setPower(-power);
            DcleftBack.setPower(-power);
            DcrightFront.setPower(power);
            DcrightBack.setPower(power);
            telemetry.addData("Yaw while turning left", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            // telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }


        DcleftFront.setPower(0);
        DcleftBack.setPower(0);
        DcrightFront.setPower(0);
        DcrightBack.setPower(0);
        //sleep(5000);
    }

    public void turnRight( DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack,
                           double degrees, double power) {
        //public void turnLeft(YawPitchRollAngles orientation, DcMotor DcleftFront, DcMotor DcrightFront, DcMotor DcleftBack, DcMotor DcrightBack,
        //                   double degrees, double power) {

        //myRobot.setDriveTrainMode(DcleftFront, DcrightFront, DcleftBack, DcrightBack, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.resetYaw();

        //orientation=imu.getRobotYawPitchRollAngles();
        //telemetry.addData("Yaw outside loop", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Yaw before while loop for right turn", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Right turn degree value ", degrees);
        telemetry.update();
        //sleep(5000);

        while(opModeIsActive() && (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > -degrees)) {
            // while(opModeIsActive() && (orientation.getYaw(AngleUnit.DEGREES) < degrees)) {
            //while(opModeIsActive()) {
            DcleftFront.setPower(power);
            DcleftBack.setPower(power);
            DcrightFront.setPower(-power);
            DcrightBack.setPower(-power);
            telemetry.addData("Yaw while turning right", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            // telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        DcleftFront.setPower(0);
        DcleftBack.setPower(0);
        DcrightFront.setPower(0);
        DcrightBack.setPower(0);

        telemetry.addData("final", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
        //sleep(5000);
    }

    public void outtakeArmUpV2 (Servo elbow, Servo wrist){
//        if (outtakeArmUp == false){
//            elbow.setPosition(0.73);
//            mStateTime.reset();
//            outtakeArmUp = true;
//        }
//        if (outtakeArmUp == true && mStateTime.time() >= 0.1){
//            wrist.setPosition(0.1);
//            outtakeArmUp = false;
//        }

        elbow.setPosition(0.73);
        mStateTime.reset();

        while (mStateTime.time() <= 0.1){

        }

        wrist.setPosition(0.1);
    }
}